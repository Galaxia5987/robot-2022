package frc.robot.subsystems.shooter.commands.bits;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commandgroups.ShootCargo;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.BooleanSupplier;

public class TryVelocities extends SequentialCommandGroup {
    private final BooleanSupplier buttonRunUnsuccessful;
    private final BooleanSupplier buttonRunSuccessful;
    private final BooleanSupplier shoot;
    private final Conveyor conveyor;
    private final Shooter shooter;
    private final Hood hood;
    private final Flap flap;
    private final double distanceFromTarget;
    private boolean lastButtonRunUnsuccessful;
    private boolean lastButtonRunSuccessful;
    private boolean lastButtonShoot;
    private boolean addShot = false;
    private double shooterVelocity;

    public TryVelocities(Conveyor conveyor, Shooter shooter, Hood hood, Flap flap,
                         BooleanSupplier buttonRunSuccessful, BooleanSupplier buttonRunUnsuccessful, BooleanSupplier shoot,
                         double distanceFromTarget) {
        this.buttonRunUnsuccessful = buttonRunUnsuccessful;
        this.buttonRunSuccessful = buttonRunSuccessful;
        this.shoot = shoot;
        this.conveyor = conveyor;
        this.shooter = shooter;
        this.hood = hood;
        this.flap = flap;
        this.distanceFromTarget = distanceFromTarget;
        addCommandsForNextVelocity();
    }

    @Override
    public void execute() {
        super.execute();
        lastButtonRunUnsuccessful = buttonRunUnsuccessful.getAsBoolean();
        lastButtonRunSuccessful = buttonRunSuccessful.getAsBoolean();
        lastButtonShoot = shoot.getAsBoolean();

        if (getOutputs()[2]) {
            addShot = !addShot;
        }
        if (addShot) {
            addCommandsForNextVelocity();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        System.out.println("The velocity that got to the target is " + shooterVelocity);
    }

    @Override
    public boolean isFinished() {
        return getOutputs()[1];
    }

    private boolean[] getOutputs() {
        var toggleRunUnsuccessful = !lastButtonRunUnsuccessful && buttonRunUnsuccessful.getAsBoolean();
        var toggleRunSuccessful = !lastButtonRunSuccessful && buttonRunSuccessful.getAsBoolean();
        var toggleShoot = !lastButtonShoot && shoot.getAsBoolean();

        return new boolean[]{toggleRunUnsuccessful, toggleRunSuccessful, toggleShoot};
    }

    private void addCommandsForNextVelocity() {
        BooleanSupplier isFinished = () -> (getOutputs()[0] || getOutputs()[1]);

        addCommands(
                new ShootCargo(shooter, hood, conveyor, flap,
                        Constants.Conveyor.DEFAULT_POWER::get, () -> distanceFromTarget, () -> true, () -> 0)
                        .withInterrupt(() -> getOutputs()[2]),
                new WaitUntilCommand(isFinished).andThen(() -> {
                    if (getOutputs()[0]) {
                        shooterVelocity += 50;
                    }
                })
        );
    }
}
