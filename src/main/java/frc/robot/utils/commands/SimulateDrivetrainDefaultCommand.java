package frc.robot.utils.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.SimulateDrivetrain;
import frc.robot.utils.Utils;

public class SimulateDrivetrainDefaultCommand extends CommandBase {
    private final XboxController xbox;
    private final SimulateDrivetrain simulateDrivetrain;

    public SimulateDrivetrainDefaultCommand(
            XboxController xbox, SimulateDrivetrain simulateDrivetrain) {
        this.xbox = xbox;
        this.simulateDrivetrain = simulateDrivetrain;
        addRequirements(simulateDrivetrain);
    }

    @Override
    public void execute() {
        simulateDrivetrain.set(Utils.deadband(-xbox.getLeftY(), 0.1),
                Utils.deadband(xbox.getRightX(), 0.1));
    }
}
