package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.commands.AdjustAngle;
import frc.robot.subsystems.climber.commands.JoystickClimb;
import frc.robot.subsystems.climber.commands.StopClimber;
import frc.robot.utils.PhotonVisionModule;
import frc.robot.utils.SimulateDrivetrain;
import frc.robot.utils.commands.SimulateDrivetrainDefaultCommand;
import webapp.Webserver;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xbox, XboxController.Button.kB.value);
    private final JoystickButton y = new JoystickButton(xbox, XboxController.Button.kY.value);
    // The robot's subsystems and commands are defined here...
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
        a.toggleWhenPressed(new JoystickClimb(climber, () -> joystick.getRawAxis(XboxController.Axis.kLeftX.value)));
        b.toggleWhenPressed(new StopClimber(climber));

        a.and(b).and(y).toggleWhenActive(new AdjustAngle(climber));
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
