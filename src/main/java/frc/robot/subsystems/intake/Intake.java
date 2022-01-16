package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Intake extends SubsystemBase {
    public static Intake INSTANCE;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Intake.MOTOR_PORT);
    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Intake.SOLENOID_PORT);

    public Intake() {
        motor.setInverted(Ports.Intake.MOTOR_INVERTED);
    }


    /**
     * lazy instantiation
     *
     * @return the subsystem instance
     */
    public static Intake getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Intake();
        }
        return INSTANCE;
    }

    /**
     * sets the power of the intake
     *
     * @param power desired power
     */
    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    /**
     * open intake solenoid
     */
    public void openSolenoid() {
        solenoid.set(Constants.Intake.SOLENOID_OPEN);
    }

    /**
     * close intake solenoid
     */
    public void closeSolenoid() {
        solenoid.set(Constants.Intake.SOLENOID_CLOSE);
    }

    /**
     * toggles intake solenoid
     */
    public void toggleSolenoid() {
        solenoid.toggle();
    }
}
