package frc.robot.subsystems.hood.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hood.Hood;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class HoodCommand extends CommandBase {
    private final Hood hood;
    private final BooleanSupplier postFlap;
    private final DoubleSupplier distance;
    private final Timer timer = new Timer();
    private final boolean last = false;
    private Hood.Mode mode = Hood.Mode.ShortDistance;
    private boolean starting = true;

    public HoodCommand(Hood hood, BooleanSupplier postFlap, DoubleSupplier distance) {
        this.hood = hood;
        this.postFlap = postFlap;
        this.distance = distance;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
//        if (hasTarget.getAsBoolean()) {
        mode = RobotContainer.cachedDistance < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance;
//        } else {
//            mode = odometryDistance.getAsDouble() < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance;
//        }
    }

    @Override
    public void execute() {
        hood.setSolenoid(mode);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        starting = true;
    }
}
