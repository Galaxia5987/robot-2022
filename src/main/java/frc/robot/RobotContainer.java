package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeByRobotSpeed;
import frc.robot.subsystems.intake.commands.IntakeByVision;
import frc.robot.valuetuner.ValueTuner;
import webapp.Webserver;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Intake intake = Intake.getInstance();
    public ExampleSubsystem exampleSubsystem = ExampleSubsystem.getInstance();
    private final XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xbox, XboxController.Button.kB.value);
    private final JoystickButton x = new JoystickButton(xbox, XboxController.Button.kX.value);
    private final JoystickButton y = new JoystickButton(xbox, XboxController.Button.kY.value);

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
    }

    private void configureButtonBindings() {
        b.whileHeld(new IntakeByRobotSpeed(intake, () -> Constants.Intake.POWER));
        x.whileHeld(new IntakeByVision(intake, () -> true, Constants.Intake.POWER));
        y.whenPressed(intake::toggleRetractor);
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
