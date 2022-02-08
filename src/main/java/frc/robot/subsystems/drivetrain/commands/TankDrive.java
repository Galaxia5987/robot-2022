package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.DoubleSupplier;

public class TankDrive extends CommandBase {
    private final DoubleSupplier rightVelocity;
    private final DoubleSupplier leftVelocity;
    private final SwerveDrive swerveDrive;

    public TankDrive(SwerveDrive swerveDrive, DoubleSupplier rightVelocity, DoubleSupplier leftVelocity) {
        this.swerveDrive = swerveDrive;
        this.rightVelocity = rightVelocity;
        this.leftVelocity = leftVelocity;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double rightForward = rightVelocity.getAsDouble();
        double leftForward = leftVelocity.getAsDouble();
        SwerveModuleState rightState = new SwerveModuleState(rightForward, new Rotation2d(0));
        SwerveModuleState leftState = new SwerveModuleState(leftForward, new Rotation2d(0));

        swerveDrive.getModule(0).setState(rightState);
        swerveDrive.getModule(2).setState(rightState);

        swerveDrive.getModule(1).setState(leftState);
        swerveDrive.getModule(3).setState(leftState);
    }
}