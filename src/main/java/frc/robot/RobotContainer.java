package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.hood.Hood;
import frc.robot.utils.PhotonVisionModule;
import frc.robot.utils.SimulateDrivetrain;

import webapp.Webserver;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Intake intake = Intake.getInstance();
    private final XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xbox, XboxController.Button.kB.value);
    private final Hood hood = Hood.getInstance();
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
    }

    private void configureButtonBindings() {
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
