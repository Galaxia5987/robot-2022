package frc.robot.command_groups;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.FeedToShooter;
import frc.robot.subsystems.shooter.commands.PrepareShooter;
import frc.robot.subsystems.shooter.commands.FeedToShooter;

import java.util.function.DoubleSupplier;

public class Shoot extends SequentialCommandGroup {
    private Shooter shooter;
    private DoubleSupplier distance;

    public Shoot(Shooter shooter, DoubleSupplier distance) {
        this.shooter = shooter;
        this.distance = distance;
        /*
        Once the rest of the robot is operational, another command will
        be added to feed the balls to the shooter.
         */
        addCommands(
                new PrepareShooter(shooter, () -> Shooter.getSetpointVelocity(-distance.getAsDouble()))
        );
    }

//    @Override
//    public void execute() {
//        if(shooter.getVelocity() == Shooter.getSetpointVelocity(distance.getAsDouble())){
//            new FeedToShooter();
//        }
//    }
}
