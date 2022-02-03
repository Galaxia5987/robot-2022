package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SwerveDrive.ROTATION_MULTIPLIER;
import static frc.robot.Constants.SwerveDrive.VELOCITY_MULTIPLIER;

// AssistedDrive, WaitBeforeAccel, DriveSlowAccel
public class OverpoweredDrive extends CommandBase {
    PIDController pidController = new PIDController(0.05, 0, 0) {{
        enableContinuousInput(-180, 180);
        setTolerance(5);
    }};
    private boolean keepSetpoint = false;
    private Rotation2d setpoint = new Rotation2d();
    private boolean wait = false;
    private double current = 0;
    private final SwerveDrive swerveDrive;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;
    private final Timer timer = new Timer();

    public OverpoweredDrive(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        this.swerveDrive = swerveDrive;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        double forward = -forwardSupplier.getAsDouble();
        double strafe = -strafeSupplier.getAsDouble();
        System.out.println(forward + " " + strafe);
        double magnitude = Math.hypot(forward, strafe);
        double alpha = Math.atan2(strafe, forward);
        if (Math.abs(magnitude) < 0.1) magnitude = 0;
        if (magnitude == 0) current = 0;
        magnitude *= VELOCITY_MULTIPLIER;
        current += magnitude / 10;
        if (current > magnitude) current = magnitude;
        forward = Math.cos(alpha) * current;
        strafe = Math.sin(alpha) * current;
        double rotation = -rotationSupplier.getAsDouble();
        if (Math.abs(rotation) < 0.1) rotation = 0;
        rotation *= ROTATION_MULTIPLIER;
        if (magnitude == 0 && rotation == 0) {
            swerveDrive.terminate();
            keepSetpoint = false;
            wait = true;
            timer.reset();
        } else {
            if (rotation != 0) {
                keepSetpoint = false;
            } else if (!keepSetpoint || timer.get() <= 0.1) {
                setpoint = Robot.getAngle();
                keepSetpoint = true;
            }

            if (swerveDrive.haveReachedAngles(forward, strafe, rotation))//comment this out
                wait = false;//comment this out
            if (wait) {//comment this out
                swerveDrive.noSpeedHolonomicDrive(forward, strafe, rotation);//comment this out
            } else {//comment this out
//                if (keepSetpoint) {//comment in
//                    swerveDrive.holonomicDrive(forward, strafe, pidController.calculate(Robot.getAngle().getDegrees(), setpoint.getDegrees()));//comment in
//                } else {//comment in
                swerveDrive.holonomicDrive(forward, strafe, rotation * (1 + magnitude / (VELOCITY_MULTIPLIER * 2)));//comment this in
//                    swerveDrive.holonomicDrive(forward, strafe, rotation);
//                }//comment in
            }
        }//comment this out
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
        timer.stop();
    }
}
