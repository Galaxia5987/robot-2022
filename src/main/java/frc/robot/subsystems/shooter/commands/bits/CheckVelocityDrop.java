package frc.robot.subsystems.shooter.commands.bits;

import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import webapp.FireLog;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class CheckVelocityDrop extends Shoot {
    private double lastVelocity;
    private double maximalDrop = 0;

    public CheckVelocityDrop(Shooter shooter, Hood hood, DoubleSupplier distance, BooleanSupplier postFlap) {
        super(shooter, hood, distance, postFlap);
    }

    @Override
    public void execute() {
        super.execute();
        double velocity = shooter.getVelocity();
        double setpoint = getSetpointVelocity(distance.getAsDouble(), hood.isOpen());
        maximalDrop = Math.max(maximalDrop, lastVelocity - velocity);

        FireLog.log("Setpoint", setpoint);
        FireLog.log("Actual velocity", velocity);
        FireLog.log("Maximal drop", maximalDrop);
        FireLog.log("Percent drop", (maximalDrop / setpoint) * 100);

        lastVelocity = velocity;
    }
}
