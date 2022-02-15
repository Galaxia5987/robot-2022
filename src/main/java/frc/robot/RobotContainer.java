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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commandgroups.PickUpCargo;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.commands.AdjustAngle;
import frc.robot.subsystems.climber.commands.StopClimber;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.HolonomicDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.flap.commands.FlapCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeCargo;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import webapp.Webserver;

import java.util.OptionalDouble;

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
    private final Flap flap = Flap.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    // The robot's subsystems and commands are defined here...
//    private final SwerveDrive swerve = SwerveDrive.getFieldOrientedInstance();
//    private final Climber climber = Climber.getInstance();
   private final Conveyor conveyor = Conveyor.getInstance();
    private final Intake intake = Intake.getInstance();


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings and default commands
        configureDefaultCommands();

        if (Robot.debug) {
//            startFireLog();
        }

        configureButtonBindings();
    }

    private void configureDefaultCommands() {
//        swerve.setDefaultCommand(new HolonomicDrive(swerve, xbox::getLeftY, () -> -xbox.getLeftX(), xbox::getRightX));
//        flap.setDefaultCommand(new FlapCommand(flap, Flap.FlapMode.Open));

    }

    private void configureButtonBindings() {
//        a.whenPressed((Runnable) Robot::resetAngle);
//        b.toggleWhenPressed(new StopClimber(climber));
//
//        a.and(b).and(y).toggleWhenActive(new AdjustAngle(climber));
//        a.whileHeld(new Convey(conveyor, Constants.Conveyor.DEFAULT_POWER));
        b.whileHeld(new Convey(conveyor, -0.3));
//        x.whileHeld(new PickUpCargo(conveyor, intake, Constants.Conveyor.DEFAULT_POWER, Constants.Intake.DEFAULT_POWER));
        a.whileHeld(new ParallelCommandGroup(
                new Shoot(shooter, () -> 8, OptionalDouble.of(0.5)),
                new Convey(conveyor, Constants.Conveyor.DEFAULT_POWER))
        );
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
        return  null;
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
