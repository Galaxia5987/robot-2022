package frc.robot.subsystems.Climber.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

public class JoystickClimb extends CommandBase {
    private final Climber climber;
    private final DoubleSupplier supplier;

    public JoystickClimb(Climber climber, DoubleSupplier supplier) {
        this.climber = climber;
        this.supplier = supplier;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        double drive = supplier.getAsDouble() * Constants.Climber.MAX_VELOCITY;
        climber.setVelocity(Utils.deadband(drive, Constants.Climber.THRESHOLD));
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
