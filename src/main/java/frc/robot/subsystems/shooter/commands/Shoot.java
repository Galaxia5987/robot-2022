package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import webapp.FireLog;

import java.util.HashMap;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class Shoot extends CommandBase {
    protected final Shooter shooter;
    protected final Hood hood;
    protected final DoubleSupplier distance;
    private final boolean bool;
    private final OptionalDouble power;
    private final Timer timer = new Timer();
    private double setpointVelocity = 0;

    public Shoot(Shooter shooter, Hood hood, double power) {
        this.shooter = shooter;
        this.hood = hood;
        this.distance = () -> 8;
        this.power = OptionalDouble.of(power);
        bool = false;
        addRequirements(shooter);
    }

    public Shoot(Shooter shooter, Hood hood, DoubleSupplier distance, boolean bool) {
        this.shooter = shooter;
        this.hood = hood;
        this.distance = distance;
        this.bool = bool;
        this.power = OptionalDouble.empty();
        addRequirements(shooter);
    }

    public Shoot(Shooter shooter, Hood hood, DoubleSupplier distance) {
        this(shooter, hood, distance, false);
    }

    /**
     * Calculates the velocity setpoint according to the distance from the target.
     * Once the data from the shooter is acquired this function will be changed.
     *
     * @param distance is the distance from the target. [m]
     * @return 15. [rpm]
     */
    public static double getSetpointVelocity(double distance, boolean isShort) {
        HashMap<Double, Double> measurements;
        if (isShort) {
            measurements = Constants.Shooter.SHORT_MEASUREMENTS;
        } else {
            measurements = Constants.Shooter.LONG_MEASUREMENTS;
        }
        double prevMeasuredDistance = 0, nextMeasuredDistance = 0;
        double minPrevDifference = Double.POSITIVE_INFINITY, minNextDifference = Double.POSITIVE_INFINITY;

        for (var measuredDistance : measurements.keySet()) {
            double difference = measuredDistance - distance;
            if (difference < 0) {
                if (Math.abs(difference) < Math.abs(minPrevDifference)) {
                    minPrevDifference = difference;
                    prevMeasuredDistance = measuredDistance;
                }
            } else {
                if (Math.abs(difference) < Math.abs(minNextDifference)) {
                    minNextDifference = difference;
                    nextMeasuredDistance = measuredDistance;
                }
            }
        }
        double y1 = measurements.get(prevMeasuredDistance);
        double y2 = measurements.get(nextMeasuredDistance);
        double t = (distance - prevMeasuredDistance) / (nextMeasuredDistance - prevMeasuredDistance);
        return (1 - t) * y1 + t * y2;
    }


    @Override
    public void initialize() {
        RobotContainer.ledSubsystem.setNeutralMode(false);
        timer.start();
        timer.reset();
        if (bool) {
            setpointVelocity = getSetpointVelocity(distance.getAsDouble(), hood.isOpen());
        } else {
            setpointVelocity = distance.getAsDouble();
        }
    }

    @Override
    public void execute() {

//        if (postFlap.getAsBoolean()) {
//            if (!last) {
//                timer.reset();
//            }
//            last = true;
//        } else {
//            last = false;
//        }

//        if (timer.hasElapsed(0.5) || starting) {
//            setpointVelocity = getSetpointVelocity(distance.getAsDouble(), hood.isOpen());
//            starting = false;
//        }
//        setpointVelocity = getSetpointVelocity(distance.getAsDouble(), hood.isOpen());


        if (power.isEmpty()) {
//            System.out.println("Distance: " + distance.getAsDouble() + ", Velocity: " + getSetpointVelocity(distance.getAsDouble(), hood.isOpen()));
            shooter.setVelocity(setpointVelocity);
            SmartDashboard.putString("speed_state", Math.abs(setpointVelocity - shooter.getVelocity()) <= 30 ? "green" : Math.abs(setpointVelocity - shooter.getVelocity()) <= 100 ? "yellow" : "red");
        } else {
            shooter.setPower(power.getAsDouble());
        }
        SmartDashboard.putNumber("something_velocity", shooter.getVelocity());
        FireLog.log("Shooter velocity", shooter.getVelocity());
        FireLog.log("Shooter setpoint", setpointVelocity);
        if (setpointVelocity != 0) {
            RobotContainer.ledSubsystem.setPercent((int) Math.round((MathUtil.clamp(shooter.getVelocity(), 0, setpointVelocity) / setpointVelocity) * 9));
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
        timer.stop();
//        starting = true;
        RobotContainer.ledSubsystem.setNeutralMode(true);
        RobotContainer.ledSubsystem.setPercent(0);
        SmartDashboard.putString("speed_state", "red");
    }
}
