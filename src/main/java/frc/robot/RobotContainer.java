package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandgroups.Outtake;
import frc.robot.commandgroups.PickUpCargo;
import frc.robot.commandgroups.ShootCargo;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveAndAdjustWithVision;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;
import webapp.Webserver;

import java.util.function.DoubleSupplier;

public class RobotContainer {
    private static final Joystick joystick = new Joystick(Ports.Controls.JOYSTICK);
    private static final Joystick joystick2 = new Joystick(Ports.Controls.JOYSTICK2);
    final PhotonVisionModule photonVisionModule = new PhotonVisionModule("photonvision", null);
    // The robot's subsystems and commands are defined here...
    private final XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xbox, XboxController.Button.kB.value);
    private final JoystickButton x = new JoystickButton(xbox, XboxController.Button.kX.value);
    private final JoystickButton y = new JoystickButton(xbox, XboxController.Button.kY.value);
    private final JoystickButton lb = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value);
    private final JoystickButton leftTrigger = new JoystickButton(joystick, Joystick.ButtonType.kTrigger.value);
    private final JoystickButton rightTrigger = new JoystickButton(joystick2, Joystick.ButtonType.kTrigger.value);
    //    private final JoystickButton rightButton = new JoystickButton(joystick2, 6);
    private final Trigger rt = new Trigger(() -> xbox.getRightTriggerAxis() > Constants.Control.RIGHT_TRIGGER_DEADBAND);
    private final Trigger lt = new Trigger(() -> xbox.getLeftTriggerAxis() > Constants.Control.RIGHT_TRIGGER_DEADBAND);
    // The robot's subsystems and commands are defined here...
    private final SwerveDrive swerve = SwerveDrive.getFieldOrientedInstance();
    private final Intake intake = Intake.getInstance();
    private final Conveyor conveyor = Conveyor.getInstance();
    private final Flap flap = Flap.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();
//    private final DigitalOutput digitalOutput = new DigitalOutput(4);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings and default commands
        configureDefaultCommands();

        if (Robot.debug) {
            startFireLog();
        }

        configureButtonBindings();
    }

    private void configureDefaultCommands() {
//        swerve.setDefaultCommand(new OverpoweredDrive(swerve, () -> -joystick.getY(), () -> -joystick.getX(), () -> -joystick2.getX()));
        swerve.setDefaultCommand(new DriveAndAdjustWithVision(swerve, () -> -joystick.getY(), () -> -joystick.getX(), () -> -joystick2.getX(), () -> photonVisionModule.getYaw().orElse(0), () -> rightTrigger.get(), () -> photonVisionModule.getDistance().orElse(0)));
//        swerve.setDefaultCommand(new HolonomicDrive(swerve, () -> -joystick.getY(), () -> -joystick.getX(), () -> -joystick2.getX()));
//        swerve.setDefaultCommand(new DriveForward(swerve));
    }

    private void configureButtonBindings() {
        DoubleSupplier distanceFromTarget = () -> photonVisionModule.getDistance().orElse(-Constants.Vision.TARGET_RADIUS) + Constants.Vision.TARGET_RADIUS;
//        DoubleSupplier conveyorPower = Constants.Conveyor.DEFAULT_POWER::get;
//        DoubleSupplier setpointVelocity = () -> Shoot.getSetpointVelocity(distanceFromTarget.getAsDouble(), hood.isOpen());
//        BooleanSupplier isFlywheelAtSetpoint = () -> Math.abs(setpointVelocity.getAsDouble() - shooter.getVelocity()) < SHOOTER_VELOCITY_DEADBAND.get();
//        b.whenPressed(new FlapForShooting(flap, isFlywheelAtSetpoint, () -> !conveyor.isPreFlapBeamConnected()));
        rt.whileActiveContinuous(new ShootCargo(shooter, hood, conveyor, flap, () -> Constants.Conveyor.SHOOT_POWER, distanceFromTarget));
        lt.whileActiveContinuous(new ParallelCommandGroup(
                new Convey(conveyor,Constants.Conveyor.DEFAULT_POWER.get()),
                new InstantCommand(() -> flap.setFlapMode(Flap.FlapMode.STOP_CARGO))));
        b.whileHeld(new Convey(conveyor, -Constants.Conveyor.DEFAULT_POWER.get()));
        x.whenPressed(photonVisionModule::toggleLeds);
        leftTrigger.whenPressed((Runnable) Robot::resetAngle);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        photonVisionModule.setLeds(false);
        var thetaController = new ProfiledPIDController(Constants.Autonomous.KP_THETA_CONTROLLER, 0, 0, Constants.SwerveDrive.HEADING_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("p2 - Taxi from low right tarmac and pickup low cargo(7.1)", Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL);
//        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Meter", Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL);
        Robot.resetAngle(trajectory.getInitialState().holonomicRotation);
        swerve.resetOdometry(trajectory.getInitialState().poseMeters, trajectory.getInitialState().holonomicRotation);
/*        SwerveModuleState[] zeroStates = new SwerveModuleState[4];
        Arrays.fill(zeroStates, new SwerveModuleState());


        return new SequentialCommandGroup(
                new FollowPath(
                        trajectory,
                        swerve::getPose,
                        swerve.getKinematics(),
                        new PIDController(Constants.Autonomous.KP_X_CONTROLLER, 0, 0),
                        new PIDController(Constants.Autonomous.KP_Y_CONTROLLER, 0, 0),
                        swerve::setStates,
                        swerve)
        );*/

//        return new TaxiFromLowRightPickShootPickShoot(shooter, swerve, conveyor, intake, hood, flap, photonVisionModule);
        return null;
    }

    /**
     * Initiates the value tuner.
     * <p>
     * Initiates the port of team 225s Fire-Logger.
     */
    private void startFireLog() {
        try {
            new Webserver();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
