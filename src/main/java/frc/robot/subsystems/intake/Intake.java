package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Intake extends SubsystemBase {
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Intake.MOTOR_PORT);
    public Intake() {
        motor.setInverted(Ports.Intake.MOTOR_INVERTED);
    }
    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }
    public boolean isAllianceColor(String allianceColor){
        String temporaryBallColor = "Blue";
        if (temporaryBallColor.equals(allianceColor)){
            return true;
        }else
            return false;
    }
}
