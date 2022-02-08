package frc.robot.subsystems.conveyor.commands.bits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;

import java.util.function.DoubleSupplier;

public class CheckColorSensor extends Convey {
    private final DoubleSupplier direction;
    private double lastDirection;
    private double maximalNumberOfBalls = 0;
    private final Flap flap;

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

        if(currentDirection > 0) {
            SmartDashboard.putString("First", queue.getFirst());
            SmartDashboard.putString("Last", queue.getLast());
            SmartDashboard.putNumber("Number of cargo", conveyor.getCargoCount());
        } else if(currentDirection < 0) {
            if (currentDirection != lastDirection) {
                maximalNumberOfBalls = conveyor.getCargoCount();
            } else {
                SmartDashboard.putString("First", queue.getFirst());
                SmartDashboard.putString("Last", queue.getLast());
                SmartDashboard.putNumber("Number of cargo", conveyor.getCargoCount());
            }
        } else if(maximalNumberOfBalls > 0) {
            SmartDashboard.putNumberArray("[In, Out]",
                    new double[]{conveyor.getCargoCount(), maximalNumberOfBalls});
        }

        lastDirection = currentDirection;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        flap.openFlap();
    }
}
