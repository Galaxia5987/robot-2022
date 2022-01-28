package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.HolonomicDrive;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.valuetuner.ValueTuner;
import webapp.Webserver;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static XboxController xbox = new XboxController(Ports.Controls.XBOX);
    public static Joystick joystick = new Joystick(2);
    public static Joystick joystick2 = new Joystick(3);
    private final SwerveDrive swerve = new SwerveDrive(true);


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings and default commands
        configureDefaultCommands();

        if (Robot.debug) {
            startValueTuner();
            startFireLog();
        }

        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new HolonomicDrive(swerve, () -> joystick.getY(), () -> joystick.getX(), () -> joystick2.getX()));
    }

    private void configureButtonBindings() {
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        PathPlannerTrajectory path = PathPlanner.loadPath("Path", 3, 1.5, false);
        swerve.resetOdometry(new Pose2d(path.getInitialState().poseMeters.getTranslation(), path.getInitialState().holonomicRotation), path.getInitialState().holonomicRotation);
        Robot.resetAngle(path.getInitialState().holonomicRotation);
        return new PPSwerveControllerCommand(
                path,
                swerve::getPose,
                swerve.getKinematics(),
                new PIDController(2, 0, 0),
                new PIDController(2, 0, 0),
                new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(3, 1.5)) {{
                    enableContinuousInput(-Math.PI, Math.PI);
                }},
                swerve::setStates
        );
    }

    /**
     * Initiates the value tuner.
     */
    private void startValueTuner() {
        new ValueTuner().start();
    }

    /**
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
