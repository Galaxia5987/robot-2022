package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import frc.robot.subsystems.drivetrain.commands.DriveSlowAccel;
import frc.robot.subsystems.drivetrain.commands.HolonomicDrive;
import frc.robot.subsystems.drivetrain.commands.OverpoweredDrive;
import webapp.Webserver;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static XboxController xbox = new XboxController(Ports.Controls.XBOX);
    public static Joystick joystick = new Joystick(Ports.Controls.JOYSTICK);
    public static Joystick joystick2 = new Joystick(Ports.Controls.JOYSTICK2);
    private final SwerveDrive swerve = SwerveDrive.getFieldOrientedInstance();
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final JoystickButton leftTrigger = new JoystickButton(joystick, Joystick.ButtonType.kTrigger.value);

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
        swerve.setDefaultCommand(new OverpoweredDrive(swerve, () -> joystick.getY(), () -> joystick.getX(), () -> joystick2.getX()));
//        swerve.setDefaultCommand(new OverpoweredDrive(swerve, () -> xbox.getLeftY(), () -> xbox.getLeftX(), () -> xbox.getRightX()));
//        swerve.setDefaultCommand(new HolonomicDrive(swerve, () -> xbox.getLeftY(), () -> xbox.getLeftX(), () -> xbox.getRightX()));
//        swerve.setDefaultCommand(new DriveSlowAccel(swerve, () -> joystick.getY(), () -> joystick.getX(), () -> joystick2.getX()));
    }

    private void configureButtonBindings() {
        a.whenPressed(() -> Robot.resetAngle());
        leftTrigger.whenPressed(() -> Robot.resetAngle());
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        PathPlannerTrajectory path = PathPlanner.loadPath("Path", Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL, false);
        swerve.resetOdometry(new Pose2d(path.getInitialState().poseMeters.getTranslation(), path.getInitialState().holonomicRotation), path.getInitialState().holonomicRotation);
        Robot.resetAngle(path.getInitialState().holonomicRotation);
        var thetaController = new ProfiledPIDController(Constants.Autonomous.kPThetaController, 0, 0, Constants.Autonomous.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new PPSwerveControllerCommand(
                path,
                swerve::getPose,
                swerve.getKinematics(),
                new PIDController(Constants.Autonomous.kPXController, 0, 0),
                new PIDController(Constants.Autonomous.kPYController, 0, 0),
                thetaController,
                swerve::setStates
        );
    }

    /**
     * Initiates the value tuner.
     * <p>
     * /**
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
