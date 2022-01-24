package frc.robot.subsystems.shooter.commands.bits;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Shooter.Bits.OUTPUT_MULTIPLIER;

public class TestSolenoidPressure extends CommandBase {
    private final DoubleSupplier joystick;
    private final Shooter shooter;

    public TestSolenoidPressure(DoubleSupplier joystick, Shooter shooter) {
        this.joystick = joystick;
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        double output = -joystick.getAsDouble();
        output *= OUTPUT_MULTIPLIER;
        shooter.setPower(output);
    }
}
