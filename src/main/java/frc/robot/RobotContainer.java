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
import frc.robot.utils.PhotonVisionModule;
import frc.robot.utils.SimulateDrivetrain;
import frc.robot.utils.commands.SimulateDrivetrainDefaultCommand;

import webapp.Webserver;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static XboxController xbox = new XboxController(Ports.Controls.XBOX);
    public static Joystick joystick = new Joystick(2);
    public static Joystick joystick2 = new Joystick(3);
    private final SwerveDrive swerve = new SwerveDrive(true);
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final SimulateDrivetrain simulateDrivetrain = new SimulateDrivetrain();
    private final PhotonVisionModule visionModule;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (Robot.isSimulation()) {
            visionModule = new PhotonVisionModule("photonvision", simulateDrivetrain);
        } else {
            visionModule = new PhotonVisionModule("photonvision", null);
        }
        // Configure the button bindings and default commands
        configureDefaultCommands();

        if (Robot.debug) {
            startFireLog();
        }

        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        simulateDrivetrain.setDefaultCommand(new SimulateDrivetrainDefaultCommand(
                xbox, simulateDrivetrain));
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
