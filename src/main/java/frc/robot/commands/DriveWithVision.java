package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SwerveDrive.ROTATION_MULTIPLIER;
import static frc.robot.Constants.SwerveDrive.SPEED_MULTIPLIER;


public class DriveWithVision extends CommandBase {
    private final SwerveDrive swerve;
    private final Vision vision;
    private final DoubleSupplier targetSupplier;
    ProfiledPIDController profiledPIDController = new ProfiledPIDController(0.8, 0, 0, new TrapezoidProfile.Constraints(5, 2.5)) {{
        enableContinuousInput(-180, 180);
        setTolerance(5);
    }};
    private double current = 0;


    public DriveWithVision(SwerveDrive swerve, Vision vision, DoubleSupplier targetSupplier) {
        this.swerve = swerve;
        this.vision = vision;
        this.targetSupplier = targetSupplier;
        addRequirements(swerve, vision);
    }

    @Override
    public void execute() {
        double targetYaw = vision.getTargetYaw();
        double forward = -RobotContainer.joystick.getY();
        double strafe = -RobotContainer.joystick.getX();
        double magnitude = Math.hypot(forward, strafe);
        double alpha = Math.atan2(strafe, forward);
        if (Math.abs(magnitude) < 0.1)
            magnitude = 0;
        if (magnitude == 0) current = 0;
        magnitude *= SPEED_MULTIPLIER;
        current += magnitude / 50;
        if (current > magnitude) current = magnitude;
        forward = Math.cos(alpha) * current;
        strafe = Math.sin(alpha) * current;
        double rotation = -RobotContainer.joystick2.getX();
        if (Math.abs(rotation) < 0.1) rotation = 0;
        rotation *= ROTATION_MULTIPLIER;
        if (magnitude == 0 && rotation == 0 && !RobotContainer.joystick2.getTrigger())
            swerve.terminate();
        else {
            if (RobotContainer.joystick2.getTrigger()) {
                swerve.holonomicDrive(forward, strafe, profiledPIDController.calculate(Robot.getAngle().getDegrees(), targetSupplier.getAsDouble()));
            } else {
                swerve.holonomicDrive(forward, strafe, rotation);
            }
        }
        System.out.println((Robot.getAngle().getDegrees() + "," + targetSupplier.getAsDouble()));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.terminate();
    }
}
