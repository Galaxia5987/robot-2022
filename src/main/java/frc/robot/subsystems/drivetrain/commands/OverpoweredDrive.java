package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SwerveDrive.ROTATION_MULTIPLIER;
import static frc.robot.Constants.SwerveDrive.VELOCITY_MULTIPLIER;

// AssistedDrive, WaitBeforeAccel, DriveSlowAccel
public class OverpoweredDrive extends CommandBase {
    PIDController pidController = new PIDController(0.08, 0, 0) {{
        enableContinuousInput(-180, 180);
        setTolerance(5);
    }};
    private boolean keepSetpoint = false;
    private Rotation2d setpoint = new Rotation2d();
    private boolean wait = false;
    private double current = 0;
    SwerveDrive swerveDrive;
    DoubleSupplier forwardSupplier;
    DoubleSupplier strafeSupplier;
    DoubleSupplier rotationSupplier;

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
        } else {
            if (rotation != 0) {
                keepSetpoint = false;
            } else if (!keepSetpoint) {
                setpoint = Robot.getAngle();
                keepSetpoint = true;
            }

            if (swerveDrive.haveReached(forward, strafe, rotation))//comment this out
                wait = false;//comment this out
            if (wait) {//comment this out
                swerveDrive.noSpeedSetChassisSpeedsStateSpace(forward, strafe, rotation);//comment this out
            } else {//comment this out
                if (keepSetpoint) {
                    swerveDrive.holonomicDrive(forward, strafe, pidController.calculate(Robot.getAngle().getDegrees(), setpoint.getDegrees()));
                } else {
                    swerveDrive.holonomicDrive(forward, strafe, rotation);
                }

            }
        }//comment this out
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}
