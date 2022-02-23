package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.subsystems.hood.commands.HoodCommand;
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
        swerve.setDefaultCommand(new DriveAndAdjustWithVision(swerve, () -> -joystick.getY(), () -> -joystick.getX(), () -> -joystick2.getX(), () -> photonVisionModule.getYaw().orElse(-100), () -> rightTrigger.get(), () -> photonVisionModule.getDistance().orElse(-Constants.Vision.TARGET_WIDTH / 2)));

    }

    private void configureButtonBindings() {
        DoubleSupplier distanceSupplier = () -> photonVisionModule.getDistance().orElse(-Constants.Vision.TARGET_WIDTH / 2) + Constants.Vision.TARGET_WIDTH / 2;
        rt.whileActiveContinuous(new ShootCargo(shooter, hood, conveyor, flap, () -> Constants.Conveyor.SHOOT_POWER, distanceSupplier));
        lt.whileActiveContinuous(new PickUpCargo(conveyor, flap, intake, Constants.Conveyor.DEFAULT_POWER.get(), Constants.Intake.DEFAULT_POWER::get));
        a.whileHeld(new Outtake(intake, conveyor, flap, shooter, hood, () -> false));

        leftTrigger.whenPressed(() -> Robot.resetAngle());
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
