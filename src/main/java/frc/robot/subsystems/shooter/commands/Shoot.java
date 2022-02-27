package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import webapp.FireLog;

import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Shoot extends CommandBase {
    protected final Shooter shooter;
    protected final Hood hood;
    protected final DoubleSupplier distance;
    private final OptionalDouble power;
    private final Timer timer = new Timer();
    private final BooleanSupplier postFlap;
    private boolean last = false;
    private double setpointVelocity = 0;
    private boolean starting = true;

    public Shoot(Shooter shooter, Hood hood, double power) {
        this.shooter = shooter;
        this.hood = hood;
        this.postFlap = () -> false;
        this.distance = () -> 8;
        this.power = OptionalDouble.of(power);
        addRequirements(shooter);
    }

    public Shoot(Shooter shooter, Hood hood, DoubleSupplier distance, BooleanSupplier postFlap) {
        this.shooter = shooter;
        this.hood = hood;
        this.distance = distance;
        this.postFlap = postFlap;
        this.power = OptionalDouble.empty();
        addRequirements(shooter);
    }

    /**
     * Calculates the velocity setpoint according to the distance from the target.
     * Once the data from the shooter is acquired this function will be changed.
     *
     * @param distance is the distance from the target. [m]
     * @return 15. [rpm]
     */
    public static double getSetpointVelocity(double distance, boolean isShort) {
        if (isShort) {
            return 187.25 * Math.pow(distance, 2) - 415.01 * distance + 3736.9;
        }
        return 4.1217 * Math.pow(distance, 2) + 487.03 * distance + 1849.3;
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
        starting = true;
        setpointVelocity = getSetpointVelocity(distance.getAsDouble(), hood.isOpen());
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
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
        timer.stop();
//        starting = true;
    }
}
