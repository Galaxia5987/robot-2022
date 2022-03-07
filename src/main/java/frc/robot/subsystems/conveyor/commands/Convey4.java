package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.hood.Hood;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Shooter.SHOOTER_VELOCITY_DEADBAND;

public class Convey4 extends CommandBase {
    private final Conveyor conveyor;
    private final BooleanSupplier preFlapSupplier;
    private final Timer timer = new Timer();
    private final DoubleSupplier setpointSuppier;
    private final DoubleSupplier velocitySupplier;
    private final Hood hood;
    private final DoubleSupplier distance;
    boolean go = false;
    private boolean last = false;
    private boolean first = true;
    private boolean getBallToPreFlap = true;
    private Timer delayTimer = new Timer();
    private double setpoint = 0;
    private boolean wait = true;
    private Hood.Mode mode = Hood.Mode.ShortDistance;

    public Convey4(Conveyor conveyor, Hood hood, BooleanSupplier preFlapSupplier, DoubleSupplier distanceSupplier, DoubleSupplier velocitySupplier, DoubleSupplier distance) {
        this.conveyor = conveyor;
        this.hood = hood;
        this.preFlapSupplier = preFlapSupplier;
        this.setpointSuppier = distanceSupplier;
        this.velocitySupplier = velocitySupplier;
        this.distance = distance;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        mode = distance.getAsDouble() < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance;
        timer.stop();
        delayTimer.stop();
        if (preFlapSupplier.getAsBoolean()) {
            getBallToPreFlap = false;
        }
        setpoint = setpointSuppier.getAsDouble();
//        conveyor.setPower(Constants.Conveyor.DEFAULT_POWER.get());
        wait = true;
        last = false;
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("getBallToPreFlap", getBallToPreFlap);
        SmartDashboard.putBoolean("last", last);
        SmartDashboard.putNumber("timer", timer.get());
        SmartDashboard.putBoolean("setpoint_reached", Math.abs(setpoint - velocitySupplier.getAsDouble()) < SHOOTER_VELOCITY_DEADBAND.get());
        SmartDashboard.putBoolean("wait", wait);

        if (wait) {
            if (Math.abs(setpoint - velocitySupplier.getAsDouble()) < SHOOTER_VELOCITY_DEADBAND.get()) {
                wait = false;
//                last = false;
            }
            SmartDashboard.putString("Saar", "Mama");
        } else {
            SmartDashboard.putString("Saar", "Joe");
            SmartDashboard.putNumber("Saar2", timer.get());

            if (getBallToPreFlap) {
                conveyor.setPower(Constants.Conveyor.DEFAULT_POWER.get());
            } else {
//            if (delayTimer.hasElapsed(0.1))
                conveyor.setPower(0);
            }

            if (preFlapSupplier.getAsBoolean()) {
                if (!last) {
                    if (!conveyor.getQueue().isEmpty()) {
//                        if (!conveyor.getQueue().getFirst().equals(DriverStation.getAlliance())) {
                        if (!conveyor.getQueue().getFirst().equals(DriverStation.Alliance.Blue)) {
                            if (Hood.Mode.LongDistance != mode) {
                                hood.setSolenoid(Hood.Mode.LongDistance);
                            } else {
                                hood.setSolenoid(Hood.Mode.ShortDistance);
                            }
                        } else {
                            hood.setSolenoid(mode);
                        }
                    } else {
                        hood.setSolenoid(mode);
                    }
                    last = true;
                    getBallToPreFlap = false;
                    timer.start();
                    timer.reset();
                    delayTimer.start();
                    delayTimer.reset();
                }
            } else {
                last = false;
                getBallToPreFlap = true;
            }

//            if (Math.abs(setpoint - velocitySupplier.getAsDouble()) < SHOOTER_VELOCITY_DEADBAND.get()) {
            if (timer.hasElapsed(0.8)) {
                getBallToPreFlap = true;
                SmartDashboard.putNumber("time", timer.get());
                timer.reset();
                timer.stop();
            }
//            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        delayTimer.stop();
        conveyor.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}