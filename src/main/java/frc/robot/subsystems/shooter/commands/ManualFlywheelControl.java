package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class ManualFlywheelControl extends CommandBase {
    private final DoubleSupplier joystickY;
    private final Shooter shooter;

    public ManualFlywheelControl(DoubleSupplier joystickY, Shooter shooter) {
        this.joystickY = joystickY;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setPower(Constants.Shooter.OUTPUT_MULTIPLIER * -joystickY.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            System.out.println("I've been shanked!");
        } else {
            System.out.println("Someone tried to shank me!");
        }
        shooter.setPower(0);
    }
}
