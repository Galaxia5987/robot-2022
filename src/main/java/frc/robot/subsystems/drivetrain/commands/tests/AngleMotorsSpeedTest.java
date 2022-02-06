package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.Arrays;

public class AngleMotorsSpeedTest extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Timer timer = new Timer();
    private final double[] times = new double[4];
    private final SwerveModuleState[] states = new SwerveModuleState[4];

    public AngleMotorsSpeedTest(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        timer.start();
        Arrays.fill(states, new SwerveModuleState(0, Rotation2d.fromDegrees(90)));
    }

    @Override
    public void execute() {
        swerveDrive.setStates(states);
        for (int i = 0; i < 4; i++) {
            if (Math.abs(swerveDrive.getModule(i).getAngle().minus(Rotation2d.fromDegrees(90)).getDegrees()) < 3) {
                if (times[i] == 0) times[i] = timer.get();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        for (int i = 0; i < 4; i++) {
            System.out.println("WHEEL: " + i + " passed 90 degrees in: " + times[i] + " seconds.");
            SmartDashboard.putString("SpeedTestResult [WHEEL " + i + "]", "WHEEL: " + i + " passed 90 degrees in: " + times[i] + " seconds.");
        }
        swerveDrive.terminate();
    }

    @Override
    public boolean isFinished() {
        for (double time : times) {
            if (time == 0) return false;
        }
        return true;
    }
}
