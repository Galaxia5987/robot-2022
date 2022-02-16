package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commandgroups.PickUpCargo;
import frc.robot.commandgroups.ShootCargo;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.commands.AdjustAngle;
import frc.robot.subsystems.climber.commands.StopClimber;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.HolonomicDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.flap.commands.FlapCommand;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.HoodCommand;
import frc.robot.subsystems.hood.commands.bits.CheckHoodPressure;
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
    // The robot's subsystems and commands are defined here...
//    private final SwerveDrive swerve = SwerveDrive.getFieldOrientedInstance();
//    private final Intake intake = Intake.getInstance();
    private final Conveyor conveyor = Conveyor.getInstance();
    private final Flap flap = Flap.getInstance();
    private final Shooter shooter = Shooter.getInstance();
//    private final Climber climber = Climber.getInstance();
    private final Hood hood = Hood.getInstance();

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
//        hood.setDefaultCommand(new HoodCommand(hood, Hood.Mode.ShortDistance));
    }

    private void configureButtonBindings() {
//        b.whileHeld(new Convey(conveyor, Constants.Conveyor.DEFAULT_POWER));

    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
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
