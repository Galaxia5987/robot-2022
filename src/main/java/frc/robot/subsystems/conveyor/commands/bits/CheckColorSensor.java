package frc.robot.subsystems.conveyor.commands.bits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;

public class CheckColorSensor extends Convey {
    private final Flap flap;

    public CheckColorSensor(Flap flap, Conveyor conveyor, double power) {
        super(conveyor, () -> power);
        this.flap = flap;
        addRequirements(flap);
    }

    @Override
    public void initialize() {
        flap.allowShooting();
    }

    @Override
    public void execute() {
        super.execute();

        var queue = conveyor.getQueue();

        SmartDashboard.putString("First", queue.getFirst().name());
        SmartDashboard.putString("Last", queue.getLast().name());
        SmartDashboard.putNumber("Number of cargo", conveyor.getCargoCount());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        flap.blockShooter();
    }
}
