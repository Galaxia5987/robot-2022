package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.command_groups.Outtake;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.bits.TestPistonPressure;
import frc.robot.valuetuner.ValueTuner;
import webapp.Webserver;

import static frc.robot.Constants.Control.RIGHT_TRIGGER_DEADBAND;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public ExampleSubsystem exampleSubsystem = ExampleSubsystem.getInstance();
    private final XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final Trigger rightTrigger = new Trigger(() -> xbox.getRightTriggerAxis() > RIGHT_TRIGGER_DEADBAND);
    private final Shooter shooter = Shooter.getInstance();

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
        shooter.setDefaultCommand(new TestPistonPressure(xbox::getLeftY, shooter));
    }

    private void configureButtonBindings() {
        /*
        Currently, the shooting is at 20 meters per second. This will be changed once
        the vision becomes available for use.
         */
        rightTrigger.whileActiveContinuous(new Outtake(shooter, () -> 4));
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
