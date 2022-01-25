package frc.robot.subsystems.shooter.commands.bits;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

public class ShootAtMaximalVelocity extends CommandBase {
    private final Shooter shooter;

    public ShootAtMaximalVelocity(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        shooter.setPower(1);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
    }
}
