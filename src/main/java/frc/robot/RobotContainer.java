package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commandgroups.InterchangeableCommands;
import frc.robot.commandgroups.Outtake;
import frc.robot.commandgroups.PickUpCargo;
import frc.robot.commandgroups.ShootCargo;
import frc.robot.subsystems.conveyor.Conveyor;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.commands.IntakeByRobotSpeed;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.PhotonVisionModule;
import frc.robot.utils.SimulateDrivetrain;

import webapp.Webserver;

import java.util.OptionalDouble;

import static frc.robot.Constants.Control.LEFT_TRIGGER_DEADBAND;
import static frc.robot.Constants.Control.RIGHT_TRIGGER_DEADBAND;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();
    private final Intake intake = Intake.getInstance();
    private final SimulateDrivetrain simulateDrivetrain = new SimulateDrivetrain();
    private final PhotonVisionModule visionModule;

  // The robot's subsystems and commands are defined here...
    private final Conveyor conveyor = Conveyor.getInstance();
    private final XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xbox, XboxController.Button.kB.value);
    private final Trigger leftTrigger = new Trigger(() -> xbox.getLeftTriggerAxis() > LEFT_TRIGGER_DEADBAND);
    private final Trigger rightTrigger = new Trigger(() -> xbox.getLeftTriggerAxis() > RIGHT_TRIGGER_DEADBAND);

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
        /*
        Everywhere there is a double supplier for 8, replace it with supplier for distance from target,
        and when there is a double supplier for 4, replace it with supplier for robot velocity.
         */
        a.whileHeld(new InterchangeableCommands(
                leftTrigger::get,
                new PickUpCargo(conveyor, intake,
                        Constants.Conveyor.DEFAULT_POWER, Constants.Intake.DEFAULT_POWER),
                new IntakeByRobotSpeed(intake, () -> 4)
        ));
        b.whileHeld(
                new Outtake(
                        intake,
                        conveyor,
                        shooter,
                        Constants.Conveyor.DEFAULT_POWER * (leftTrigger.get() ? -1 : 1),
                        Outtake.getRemainingBalls(conveyor)
        ));
        rightTrigger.whenActive(new InterchangeableCommands(
                leftTrigger::get,
                new ShootCargo(shooter, conveyor, () -> 8, Constants.Conveyor.DEFAULT_POWER),
                new Shoot(shooter, () -> 8, OptionalDouble.empty())
        ));
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
