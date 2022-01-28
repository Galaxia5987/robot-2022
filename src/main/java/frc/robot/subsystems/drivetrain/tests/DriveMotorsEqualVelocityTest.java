package frc.robot.subsystems.drivetrain.tests;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveMotorsEqualVelocityTest extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Timer timer = new Timer();

    public DriveMotorsEqualVelocityTest(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        swerveDrive.setStates(new SwerveModuleState[]{
                new SwerveModuleState(3, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(3, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(3, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(3, Rotation2d.fromDegrees(0))
        });
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        double average = 0;
        double[] velocities = new double[4];
        for (int i = 0; i < 4; i++) {
            velocities[i] = swerveDrive.getModule(i).getVelocity();
            average += velocities[i];
        }
        average /= 4;
        for (int i = 0; i < 4; i++) {
            String value = "WHEEL: " + i + " has reached: " + velocities[i] + " m/s, and is off by: " + (velocities[i] - average) + "m/s from the average.";
            System.out.println(value);
            SmartDashboard.putString("SameVelocityResult [WHEEL " + i + "]", value);
        }
        swerveDrive.terminate();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(4);
    }
}
