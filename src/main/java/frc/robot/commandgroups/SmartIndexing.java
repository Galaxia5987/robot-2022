package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;


public class SmartIndexing extends CommandBase {
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final Intake intake;
    private final Flap flap;
    private final DoubleSupplier intakePower;
    private boolean outtakingShooter = false;
    private boolean outtakingIntake = false;
    private final Timer timer = new Timer();

    public SmartIndexing(Shooter shooter, Conveyor conveyor, Intake intake, Flap flap, DoubleSupplier intakePower) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.intake = intake;
        this.flap = flap;
        this.intakePower = intakePower;
        addRequirements(shooter, conveyor, intake);
    }

    @Override
    public void initialize() {
        intake.openRetractor();
    }

    @Override
    public void execute() {
        if (outtakingShooter) {
            outtakingIntake = false;
            flap.allowShooting();
            shooter.setVelocity(2000);
            intake.setPower(intakePower.getAsDouble());
            conveyor.setPower(Constants.Conveyor.DEFAULT_POWER.get());
            if (!conveyor.isPostFlapBeamConnected()) { // delay
                outtakingShooter = false;
            }
        } else if (outtakingIntake) {
            outtakingShooter = false;
            shooter.setPower(0);
            flap.blockShooter();
            conveyor.setPower(-Constants.Conveyor.DEFAULT_POWER.get());
            intake.setPower(intakePower.getAsDouble());
            if (timer.hasElapsed(0.35)) {
                timer.stop();
                outtakingIntake = false;
            }
        } else {
            shooter.setPower(0);
            flap.blockShooter();
            intake.setPower(intakePower.getAsDouble());
            conveyor.setPower(Constants.Conveyor.DEFAULT_POWER.get());
            System.out.println(conveyor.getColor().name());
            if (!conveyor.getColor().equals(DriverStation.Alliance.Blue) && !conveyor.getColor().equals(DriverStation.Alliance.Invalid) && conveyor.getColorSensorProximity() > Constants.Conveyor.MIN_PROXIMITY_VALUE) {
                if (!conveyor.isPreFlapBeamConnected()) {
                    outtakingIntake = true;
                } else {
                    outtakingShooter = true;
                }
                timer.reset();
                timer.start();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setPower(0);
        intake.setPower(0);
        flap.blockShooter();
    }
}
