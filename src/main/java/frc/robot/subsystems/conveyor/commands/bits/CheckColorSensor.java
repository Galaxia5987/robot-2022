package frc.robot.subsystems.conveyor.commands.bits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;

import java.util.function.DoubleSupplier;

public class CheckColorSensor extends Convey {
    private final Flap flap;
    private int cargoIn = 0;
    private int cargoOut = 0;

    public CheckColorSensor(Flap flap, Conveyor conveyor, double power) {
        super(conveyor, () -> power);
        this.flap = flap;
        addRequirements(flap);
    }

    @Override
    public void initialize() {
        flap.openFlap();
    }

    @Override
    public void execute() {
        super.execute();

        var queue = conveyor.getQueue();

        SmartDashboard.putString("First", queue.getFirst());
        SmartDashboard.putString("Last", queue.getLast());
        SmartDashboard.putNumber("Number of cargo", conveyor.getCargoCount());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        flap.closeFlap();
    }
}
