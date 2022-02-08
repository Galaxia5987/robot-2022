package frc.robot.subsystems.conveyor.commands.bits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;

import java.util.function.DoubleSupplier;

public class CheckColorSensor extends Convey {
    private final DoubleSupplier direction;
    private final Flap flap;
    private int cargoIn = 0;
    private int cargoOut = 0;

    public CheckColorSensor(Flap flap, Conveyor conveyor, DoubleSupplier direction, double power) {
        super(conveyor, () -> power * direction.getAsDouble());
        this.direction = direction;
        this.flap = flap;
        addRequirements(flap);
    }

    @Override
    public void initialize() {
        flap.closeFlap();
    }

    @Override
    public void execute() {
        super.execute();

        var queue = conveyor.getQueue();
        double currentDirection = -direction.getAsDouble();

        SmartDashboard.putString("First", queue.getFirst());
        SmartDashboard.putString("Last", queue.getLast());
        SmartDashboard.putNumber("Number of cargo", conveyor.getCargoCount());

        /*
        The idea behind this logic is to catch when the balls finish coming in,
        and from this we can know how many balls left (according to the sensors)
         */
        if (currentDirection > 0) {
            cargoIn = conveyor.getCargoCount();
        } else if (currentDirection < 0) {
            cargoOut = conveyor.getCargoCount();
        }

        SmartDashboard.putNumberArray("[Cargo in, Cargo out]", new double[]{cargoIn, cargoOut});
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        flap.openFlap();
    }
}
