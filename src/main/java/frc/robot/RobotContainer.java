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
import frc.robot.commandgroups.PickUpCargo;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.commands.AdjustAngle;
import frc.robot.subsystems.climber.commands.StopClimber;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.HolonomicDrive;
import frc.robot.subsystems.drivetrain.commands.OverpoweredDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utils.Utils;
import webapp.Webserver;

import java.util.function.DoubleSupplier;

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
    //    private final Climber climber = Climber.getInstance();
//    private final Conveyor conveyor = Conveyor.getInstance();
//    private final Intake intake = Intake.getInstance();


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
        DoubleSupplier joystickLeftY = () -> Utils.joystickSmoothing(-joystick.getY());
        DoubleSupplier joystickLeftX = () -> Utils.joystickSmoothing(-joystick.getX());
        DoubleSupplier joystickRightX = () -> Utils.joystickSmoothing(-joystick2.getX());
//        swerve.setDefaultCommand(new HolonomicDrive(swerve, xbox::getLeftY, () -> -xbox.getLeftX(), xbox::getRightX));
//        swerve.setDefaultCommand(new OverpoweredDrive(swerve, () -> -xbox.getLeftY(), () -> -xbox.getLeftX(), () -> -xbox.getRightX()));
        swerve.setDefaultCommand(new OverpoweredDrive(swerve, joystickLeftY, joystickLeftX, joystickRightX));
//        swerve.setDefaultCommand(new HolonomicDrive(swerve, () -> -xbox.getLeftY(), xbox::getLeftX, () -> -xbox.getRightX()));
    }

    private void configureButtonBindings() {
        leftTrigger.whenPressed((Runnable) Robot::resetAngle);
//        b.whenHeld(new PickUpCargo(conveyor, intake, 0.5, 0.5));
//        b.toggleWhenPressed(new StopClimber(climber));

//        a.and(b).and(y).toggleWhenActive(new AdjustAngle(climber));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        PathPlannerTrajectory path = PathPlanner.loadPath("Path", Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL, false);
        swerve.resetOdometry(new Pose2d(path.getInitialState().poseMeters.getTranslation(), path.getInitialState().holonomicRotation));
        Robot.resetAngle(path.getInitialState().holonomicRotation);
        var thetaController = new ProfiledPIDController(Constants.Autonomous.KP_THETA_CONTROLLER, 0, 0, Constants.SwerveDrive.HEADING_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new PPSwerveControllerCommand(
                path,
                swerve::getPose,
                swerve.getKinematics(),
                new PIDController(Constants.Autonomous.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Autonomous.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                swerve::setStates
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
