package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.valuetuner.WebConstant;

public class TestVelocity extends CommandBase {
    private final Shooter shooter;
    private final WebConstant velocity;

    public TestVelocity(Shooter shooter, WebConstant velocity) {
        this.shooter = shooter;
        this.velocity = velocity;
    }

    @Override
    public void execute() {
        shooter.setVelocity(velocity.get());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
    }
}
