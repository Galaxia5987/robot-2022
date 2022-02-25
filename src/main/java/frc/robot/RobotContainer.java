package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.OverpoweredDrive;
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
    //    private final Climber climber = Climber.getInstance();
    private final Hood hood = Hood.getInstance();
    private final PhotonVisionModule photonVisionModule = new PhotonVisionModule("photonvision", null);
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
        swerve.setDefaultCommand(new OverpoweredDrive(swerve, () -> -joystick.getY(), () -> -joystick.getX(), () -> -joystick2.getX()));
//        swerve.setDefaultCommand(new DriveForward(swerve));
    }

    private void configureButtonBindings() {
        DoubleSupplier distanceSupplier = () -> photonVisionModule.getDistance().orElse(-Constants.Vision.TARGET_WIDTH / 2) + Constants.Vision.TARGET_WIDTH / 2;
//        rt.whileActiveContinuous(new ShootCargo(shooter, hood, conveyor, flap, () -> Constants.Conveyor.SHOOT_POWER, distanceSupplier));
        x.whenPressed(intake::toggleRetractor);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
//        PathPlannerTrajectory path = PathPlanner.loadPath("Path", Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL, false);
//        swerve.resetOdometry(new Pose2d(path.getInitialState().poseMeters.getTranslation(), path.getInitialState().holonomicRotation));
//        Robot.resetAngle(path.getInitialState().holonomicRotation);
//        var thetaController = new ProfiledPIDController(Constants.Autonomous.KP_THETA_CONTROLLER, 0, 0, Constants.SwerveDrive.HEADING_CONTROLLER_CONSTRAINTS);
//        thetaController.enableContinuousInput(-Math.PI, Math.PI);
//
//        return new PPSwerveControllerCommand(
//                path,
//                swerve::getPose,
//                swerve.getKinematics(),
//                new PIDController(Constants.Autonomous.KP_X_CONTROLLER, 0, 0),
//                new PIDController(Constants.Autonomous.KP_Y_CONTROLLER, 0, 0),
//                thetaController,
//                swerve::setStates
//        );

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("why tho", 3, 1.5);
        Robot.resetAngle(trajectory.getInitialState().holonomicRotation);
        swerve.resetOdometry(trajectory.getInitialState().poseMeters, trajectory.getInitialState().holonomicRotation);
//        SmartDashboard.putNumber("trajectory_time", trajectory.getTotalTimeSeconds());
        return new PPSwerveControllerCommand(
                trajectory,
                swerve::getPose,
                swerve.getKinematics(),
                new PIDController(3, 0, 0),
                new PIDController(3, 0, 0),
                new ProfiledPIDController(7, 0, 0, new TrapezoidProfile.Constraints(4, 3.2)) {{
                    enableContinuousInput(-Math.PI, Math.PI);
                }},
                (swerve::setStates),
                swerve
        );
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
