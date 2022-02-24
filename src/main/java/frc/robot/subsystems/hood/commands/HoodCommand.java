package frc.robot.subsystems.hood.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.hood.Hood;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class HoodCommand extends CommandBase {
    private final Hood hood;
    private final BooleanSupplier postFlap;
    private final DoubleSupplier distance;
    private final Timer timer = new Timer();
    private boolean last = false;

    public HoodCommand(Hood hood, BooleanSupplier postFlap, DoubleSupplier distance) {
        this.hood = hood;
        this.postFlap = postFlap;
        this.distance = distance;
        addRequirements(hood);
    }

//    public HoodCommand(Hood hood, Supplier<Hood.Mode> modeSupplier) {
//        this.hood = hood;
//        this.modeSupplier = modeSupplier;
//        addRequirements(hood);
//    }

//    public HoodCommand(Hood hood, DoubleSupplier distance) {
//        boolean val = distance.getAsDouble() > Constants.Hood.MIN_DISTANCE ? distance.getAsDouble() < DISTANCE_FROM_TARGET_THRESHOLD :  hood.isOpen()
//        this(hood, () -> Hood.Mode.getValue(););
//    }


//    public HoodCommand(Hood hood, Hood.Mode mode) {
//        this(hood, () -> mode);
//        isInstant = true;
//    }


//    @Override
//    public void initialize() {
//        if (isInstant) {
//            execute();
//            cancel();
//        }
//    }


    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        Hood.Mode mode = distance.getAsDouble() < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance;
        if (postFlap.getAsBoolean()) {
            if (!last) {
                timer.reset();
            }
            last = true;
        } else {
            last = false;
        }
        if (timer.hasElapsed(0.5)) {
            hood.setSolenoid(mode);
        }
//        hood.setSolenoid(modeSupplier.get());
    }

    @Override
    public void end(boolean interrupted) {
//        timer.stop();
    }
}
