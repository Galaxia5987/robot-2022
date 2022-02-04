package frc.robot.subsystems.drivetrain.tests;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class HelpfulZeroing extends CommandBase {
    private final SwerveDrive swerve;
    Timer timer = new Timer();

    public HelpfulZeroing(SwerveDrive swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        SwerveModuleState[] swerveModuleStates = {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
        swerve.noOptimizeSetStates(swerveModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        timer.stop();
        swerve.terminate();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.5);
    }
}
