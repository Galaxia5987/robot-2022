package frc.robot.tests;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveMotorsSpeedTest extends CommandBase {
    SwerveDrive swerveDrive;
    Timer timer = new Timer();
    double times[] = new double[4];

    public DriveMotorsSpeedTest(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        super.initialize();
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
        for (int i = 0; i < 4; i++) {
            if (Math.abs(Math.abs(swerveDrive.getModule(i).getVelocity()) - 3) < 0.1) {
                if (times[i] == 0) times[i] = timer.get();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        for (int i = 0; i < 4; i++) {
            System.out.println("WHEEL: " + i + " reached 3m/s in: " + times[i] + " seconds.");
            SmartDashboard.putString("SpeedTestResult [WHEEL " + i + "]", "WHEEL: " + i + " reached 3m/s in: " + times[i] + " seconds.");
        }
        swerveDrive.terminate();
    }

    @Override
    public boolean isFinished() {
        for (int i = 0; i < 4; i++) {
            if (times[i] == 0) return false;
        }
        return true;
    }
}
