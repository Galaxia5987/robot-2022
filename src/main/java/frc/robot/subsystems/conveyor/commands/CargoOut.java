package frc.robot.subsystems.conveyor.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.conveyor.Conveyor;

public class CargoOut extends CommandBase {
    private final Conveyor conveyor;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Conveyor.AUX);
//    private frc.robot.subsystems.conveyor.Conveyor.AllianceColor color;

    public CargoOut(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        if (conveyor.getColorPosition(true) == DriverStation.Alliance.Blue) {
            if (conveyor.getColorPosition(false) == DriverStation.Alliance.Blue) {
                // do nothing
            }
            if (conveyor.getColorPosition(false) != DriverStation.Alliance.Blue) {
                conveyor.setPower(-Constants.POWER);
                // use InTake
            }

        }
        if (conveyor.getColorPosition(true) != DriverStation.Alliance.Blue) {
            if (conveyor.getColorPosition(false) == DriverStation.Alliance.Blue) {
                conveyor.setPower(Constants.POWER);
                //use shooter
            }
            if (conveyor.getColorPosition(false) != DriverStation.Alliance.Blue) {
                // out from any direction
            }
            if (conveyor.getColorPosition(false) == DriverStation.Alliance.Invalid) {
                conveyor.setPower(Constants.POWER);
                // use shooter - in case there is only one ball
            }
        }

    }


    @Override
    public void execute() {


    }

    // leave me alone
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}