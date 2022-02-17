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
import java.util.function.Supplier;

public class TryVelocities extends SequentialCommandGroup {
    private final BooleanSupplier wasRunUnsuccessful;
    private final BooleanSupplier wasRunSuccessful;
    private final BooleanSupplier shoot;
    private final Conveyor conveyor;
    private final Shooter shooter;
    private final Hood hood;
    private final Flap flap;
    private final double distanceFromTarget;
    private boolean lastRunUnsuccessful;
    private boolean lastRunSuccessful;
    private boolean lastShoot;
    
    private double velocity;

    public TryVelocities(Conveyor conveyor, Shooter shooter, Hood hood, Flap flap,
                         BooleanSupplier wasRunSuccessful, BooleanSupplier wasRunUnsuccessful, BooleanSupplier shoot,
                         double distanceFromTarget) {
        this.wasRunUnsuccessful = wasRunUnsuccessful;
        this.wasRunSuccessful = wasRunSuccessful;
        this.shoot = shoot;
        this.conveyor = conveyor;
        this.shooter = shooter;
        this.hood = hood;
        this.flap = flap;
        this.distanceFromTarget = distanceFromTarget;
    }

    @Override
    public void execute() {
        lastRunUnsuccessful = wasRunUnsuccessful.getAsBoolean();
        lastRunSuccessful = wasRunSuccessful.getAsBoolean();
        lastShoot = shoot.getAsBoolean();
        
        BooleanSupplier isFinished = () -> (getOutputs()[0] || getOutputs()[1]);
        
        addCommands(
                new WaitUntilCommand(() -> getOutputs()[2]),
                new ShootCargo(shooter, hood, conveyor, flap, Constants.Conveyor.DEFAULT_POWER::get, () -> distanceFromTarget, velocity),
                new WaitUntilCommand(isFinished)
        );

        if (getOutputs()[1]) {
            System.out.println("The velocity that worked is " + velocity);
            cancel();
        } else if (getOutputs()[0]) {
            velocity += 50;
        }
    }
    
    private boolean[] getOutputs() {
        var toggleRunUnsuccessful = !lastRunUnsuccessful && wasRunUnsuccessful.getAsBoolean();
        var toggleRunSuccessful = !lastRunSuccessful && wasRunSuccessful.getAsBoolean();
        var toggleShoot = !lastShoot && shoot.getAsBoolean();

        var outputRunUnsuccessful = toggleRunUnsuccessful ^ wasRunUnsuccessful.getAsBoolean();
        var outputRunSuccessful = toggleRunSuccessful ^ wasRunSuccessful.getAsBoolean();
        var outputShoot = toggleShoot ^ shoot.getAsBoolean();
        
        return new boolean[]{outputRunUnsuccessful, outputRunSuccessful, outputShoot};
    }
}
