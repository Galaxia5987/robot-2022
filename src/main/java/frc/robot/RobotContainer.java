package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autoPaths.TaxiFromLowLeftPickShoot;
import frc.robot.commandgroups.Outtake;
import frc.robot.commandgroups.PickUpCargo;
import frc.robot.commandgroups.ShootCargo;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveAndAdjustWithVision;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.helicopter.Helicopter;
import frc.robot.subsystems.helicopter.commands.JoystickPowerHelicopter;
import frc.robot.subsystems.helicopter.commands.MoveHelicopter;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeCargo;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;
import webapp.Webserver;

public class RobotContainer {
    private static final Joystick joystick = new Joystick(Ports.Controls.JOYSTICK);
    private static final Joystick joystick2 = new Joystick(Ports.Controls.JOYSTICK2);
    // The robot's subsystems and commands are defined here...
    final PhotonVisionModule photonVisionModule = new PhotonVisionModule("photonvision", null);
    private final XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xbox, XboxController.Button.kB.value);
    private final JoystickButton x = new JoystickButton(xbox, XboxController.Button.kX.value);
    private final JoystickButton y = new JoystickButton(xbox, XboxController.Button.kY.value);
    private final JoystickButton lb = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rb = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
    private final JoystickButton start = new JoystickButton(xbox, XboxController.Button.kStart.value);
    private final JoystickButton back = new JoystickButton(xbox, XboxController.Button.kBack.value);
    private final Trigger rt = new Trigger(() -> xbox.getRightTriggerAxis() > Constants.Control.RIGHT_TRIGGER_DEADBAND);
    private final Trigger lt = new Trigger(() -> xbox.getLeftTriggerAxis() > Constants.Control.RIGHT_TRIGGER_DEADBAND);
    private final Trigger upPov = new Trigger(() -> xbox.getPOV() == 0);
    private final Trigger downPov = new Trigger(() -> xbox.getPOV() == 180);
    private final Trigger rightPov = new Trigger(() -> xbox.getPOV() == 90);
    private final Trigger leftPov = new Trigger(() -> xbox.getPOV() == 270);
    private final JoystickButton leftTrigger = new JoystickButton(joystick, Joystick.ButtonType.kTrigger.value);
    private final JoystickButton rightTrigger = new JoystickButton(joystick2, Joystick.ButtonType.kTrigger.value);
    private final JoystickButton two = new JoystickButton(joystick, 2);
    private final JoystickButton twelve = new JoystickButton(joystick, 12);
    private final SwerveDrive swerve = SwerveDrive.getFieldOrientedInstance(photonVisionModule::estimatePose);
    private final Intake intake = Intake.getInstance();
    private final Conveyor conveyor = Conveyor.getInstance();
    private final Flap flap = Flap.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();
    private final Helicopter helicopter = Helicopter.getInstance();

    private double speedMultiplier = 1;

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
       swerve.setDefaultCommand(
                new DriveAndAdjustWithVision(
                        swerve,
                        () -> -joystick.getY() * speedMultiplier,
                        () -> -joystick.getX() * speedMultiplier,
                        () -> -joystick2.getX() * speedMultiplier,
                        () -> photonVisionModule.getYaw().orElse(0),
                        rightTrigger::get,
                        photonVisionModule::getDistance
                )
        );
        helicopter.setDefaultCommand(new JoystickPowerHelicopter(helicopter, () -> -xbox.getLeftY()));
//        shooter.setDefaultCommand(new Shoot(shooter, hood, WebConstant.of("Shooter", "Setpoint", 0)::get, () -> true));
    }

    private void configureButtonBindings() {
        a.whileHeld(new IntakeCargo(intake, () -> -Constants.Intake.DEFAULT_POWER.get()));
        b.whileHeld(new Convey(conveyor, Constants.Conveyor.DEFAULT_POWER.get()));
        leftPov.whileActiveOnce(new InstantCommand(hood::toggle));
        x.whenPressed(intake::toggleRetractor);
        back.whenPressed(flap::toggleFlap);
//        upPov.whileActiveOnce(new InstantCommand(hood::toggle));
        rightPov.whileActiveOnce(new InstantCommand(helicopter::toggleStopper));
        upPov.and(start).whileActiveOnce(new MoveHelicopter(helicopter, Constants.Helicopter.SECOND_RUNG));
        downPov.and(start).whileActiveOnce(new MoveHelicopter(helicopter, 0));
//        rt.whileActiveContinuous(new Shoot(shooter, hood, WebConstant.of("Shooter", "Setpoint", 0)::get, () -> true));

        rt.whileActiveContinuous(new ShootCargo(shooter, hood, conveyor, flap, () -> Constants.Conveyor.SHOOT_POWER, photonVisionModule::getDistance));
        lt.whileActiveContinuous(new PickUpCargo(conveyor, flap, intake, Constants.Conveyor.DEFAULT_POWER.get(), Constants.Intake.DEFAULT_POWER::get));
        lb.whileHeld(new Outtake(intake, conveyor, flap, shooter, hood, () -> false));
        rb.whileHeld(new Convey(conveyor, -Constants.Conveyor.DEFAULT_POWER.get()));
        start.whenPressed(photonVisionModule::toggleLeds);
        y.whenPressed(new RunCommand(() -> shooter.setVelocity(3630)).withInterrupt(rt::get));
//        twelve.whenPressed(() -> swerve.resetPoseEstimator(new Pose2d(7, 5, new Rotation2d())));


        leftTrigger.whenPressed(() -> speedMultiplier = (speedMultiplier == 0.5 ? 1 : 0.5));
        two.whenPressed((Runnable) Robot::resetAngle);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
//        return null;
        photonVisionModule.setLeds(false);

//        PathPlannerTrajectory trajectory = PathPlanner.loadPath("p2 - Taxi from low right tarmac and pickup low cargo(7.1)", Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL);
//        PathPlannerTrajectory trajectory = PathPlanner.loadPath("p2 - Taxi from low left tarmac and pickup middle cargo(6.1)", Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL);
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("p1 - Taxi from low left and pickup middle cargo(3.1)", Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL);
//        PathPlannerTrajectory trajectory = PathPlanner.loadPath("p2 - Taxi from low left tarmac and pickup middle cargo(6.1)", Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL);
        Robot.resetAngle(trajectory.getInitialState().holonomicRotation);
        swerve.resetOdometry(trajectory.getInitialState().poseMeters, trajectory.getInitialState().holonomicRotation);

//        return new TaxiFromLowRightPickShootPickShoot(shooter, swerve, conveyor, intake, hood, flap, photonVisionModule);
//        return new TaxiFromLowLeftPickShootPickShoot(shooter, swerve, conveyor, intake, hood, flap, photonVisionModule);
        return new TaxiFromLowLeftPickShoot(shooter, swerve, conveyor, intake, hood, flap, photonVisionModule);
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
