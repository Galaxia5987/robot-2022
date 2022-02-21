package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autoPaths.FiveCargoAuto;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.commands.AdjustAngle;
import frc.robot.subsystems.climber.commands.StopClimber;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.HolonomicDrive;
import frc.robot.subsystems.shooter.Shooter;
import webapp.Webserver;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final Joystick joystick = new Joystick(Ports.Controls.JOYSTICK);
    private final Joystick joystick2 = new Joystick(Ports.Controls.JOYSTICK2);
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xbox, XboxController.Button.kB.value);
    private final JoystickButton x = new JoystickButton(xbox, XboxController.Button.kX.value);
    private final JoystickButton y = new JoystickButton(xbox, XboxController.Button.kY.value);
    private final JoystickButton leftTrigger = new JoystickButton(joystick, Joystick.ButtonType.kTrigger.value);

    // The robot's subsystems and commands are defined here...
    private final SwerveDrive swerve = SwerveDrive.getFieldOrientedInstance();
    private final Climber climber = Climber.getInstance();
    private final Conveyor conveyor = Conveyor.getInstance();


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
        swerve.setDefaultCommand(new HolonomicDrive(swerve, xbox::getLeftY, () -> -xbox.getLeftX(), xbox::getRightX));
    }

    private void configureButtonBindings() {
        a.whenPressed((Runnable) Robot::resetAngle);
        b.toggleWhenPressed(new StopClimber(climber));


        a.and(b).and(y).toggleWhenActive(new AdjustAngle(climber));
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
