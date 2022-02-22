package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.OverpoweredDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.PhotonVisionModule;
import webapp.Webserver;

public class RobotContainer {
    private static final Joystick leftJoystick = new Joystick(Ports.Controls.JOYSTICK);
    private static final Joystick rightJoystick = new Joystick(Ports.Controls.JOYSTICK2);
    private final XboxController xbox = new XboxController(Ports.Controls.XBOX);

    // The robot's subsystems and commands are defined here...
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xbox, XboxController.Button.kB.value);
    private final JoystickButton x = new JoystickButton(xbox, XboxController.Button.kX.value);
    private final JoystickButton y = new JoystickButton(xbox, XboxController.Button.kY.value);
    private final Trigger xboxRightTrigger = new Trigger(() -> xbox.getRightTriggerAxis() > Constants.Control.RIGHT_TRIGGER_DEADBAND);

    private final JoystickButton joystickLeftTrigger = new JoystickButton(leftJoystick, Joystick.ButtonType.kTrigger.value);
    private final JoystickButton joystickRightTrigger = new JoystickButton(rightJoystick, Joystick.ButtonType.kTrigger.value);

    private final SwerveDrive swerve = SwerveDrive.getFieldOrientedInstance();
    private final Intake intake = Intake.getInstance();
    private final Conveyor conveyor = Conveyor.getInstance();
    private final Flap flap = Flap.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Climber climber = Climber.getInstance();
    private final Hood hood = Hood.getInstance();
    private final PhotonVisionModule photonVisionModule = new PhotonVisionModule("photonvision", null);
    private final DigitalOutput digitalOutput = new DigitalOutput(4);

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
        swerve.setDefaultCommand(new OverpoweredDrive(
                swerve, () -> -leftJoystick.getY(), () -> -leftJoystick.getX(), () -> -rightJoystick.getX()));
    }

    private void configureButtonBindings() {
        a.whileHeld(new Convey(conveyor, Constants.Conveyor.DEFAULT_POWER.get()));
        b.whenPressed(hood::toggle);
        x.whenPressed(flap::toggleFlap);
        y.whileHeld(new Convey(conveyor, -Constants.Conveyor.DEFAULT_POWER.get()));
        xboxRightTrigger.whileActiveContinuous(new Shoot(
                shooter, () -> SmartDashboard.getNumber("set_velocity", 0)));

        joystickLeftTrigger.whenPressed(() -> Robot.resetAngle());
        joystickRightTrigger.whenPressed(() -> digitalOutput.set(!digitalOutput.get()));
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
