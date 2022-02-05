package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Outtake extends CommandBase {
    private final Intake intake ;
    private final Conveyor conveyor;
    private final Shooter shooter;
    private final double conveyorPower;

    public Outtake(Intake intake, Conveyor conveyor, Shooter shooter, double conveyorPower) {
        this.intake = intake;
        this.conveyor = conveyor;
        this.shooter = shooter;
        this.conveyorPower = conveyorPower;
        addRequirements(intake, conveyor, shooter);
    }

    @Override
    public void execute() {
        if(conveyorPower < 0) {
            conveyor.setPower(conveyorPower);
            intake.setPower(conveyorPower);
        } else {
            conveyor.setFlapMode(Conveyor.FlapMode.Closed);
            conveyor.setPower(conveyorPower);
            shooter.setPower(Constants.Shooter.OUTTAKE_POWER);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setPower(0);
        intake.setPower(0);
        conveyor.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return conveyor.getCargoCount() == 0;
    }
}
