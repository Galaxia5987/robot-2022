package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commandgroups.PickUpCargo;
import frc.robot.commandgroups.ShootCargo;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.BooleanSupplier;

public class TestBallFlow extends SequentialCommandGroup {
    private final Intake intake;
    private final Shooter shooter;
    private final Conveyor conveyor;
//    private final ShootCargo shootCargo;
    private final PickUpCargo pickUpCargo;

    public TestBallFlow(Hood hood, Flap flap, Conveyor conveyor, Intake intake, Shooter shooter,
                        BooleanSupplier pickUpCargoIsFinished, BooleanSupplier shootCargoIsFinished) {
        this.intake = intake;
        this.shooter = shooter;
        this.conveyor = conveyor;
//        this.shootCargo = new ShootCargo(shooter, hood, conveyor, flap, () -> 8, () -> Constants.Conveyor.DEFAULT_POWER);
        this.pickUpCargo = new PickUpCargo(conveyor, intake, Constants.Conveyor.DEFAULT_POWER.get(), Constants.Intake.DEFAULT_POWER);

        addCommands(
//                pickUpCargo.withInterrupt(pickUpCargoIsFinished),
//                new WaitCommand(5),
//                shootCargo.withInterrupt(shootCargoIsFinished)
        );
    }

    @Override
    public void execute() {
        super.execute();

        if (pickUpCargo.isScheduled()) {
            SmartDashboard.putNumber("Intake power", intake.getPower());
//        } else if (shootCargo.isScheduled()) {
            SmartDashboard.putBoolean("Is post flap beam connected", conveyor.isPostFlapBeamConnected());
            SmartDashboard.putNumber("Shooter velocity", shooter.getVelocity());
        }

        SmartDashboard.putNumber("Conveyor power", conveyor.getPower());
        SmartDashboard.putString("First in queue", conveyor.getQueue().getFirst());
        SmartDashboard.putString("Last in queue", conveyor.getQueue().getLast());
        SmartDashboard.putNumber("Number of balls in conveyor", conveyor.getCargoCount());
        SmartDashboard.putBoolean("Is pre flap beam connected", conveyor.isPreFlapBeamConnected());
    }
}
