package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Intake extends SubsystemBase {
    private static Intake INSTANCE;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Intake.MOTOR);
    private final Solenoid retractor = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Intake.SOLENOID);

    private Intake() {
        motor.setInverted(Ports.Intake.IS_MOTOR_INVERTED);
        motor.enableVoltageCompensation(true);

    }


    /**
     * creates an instance of the intake
     *
     * @return the subsystem instance.
     */
    public static Intake getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Intake();
        }
        return INSTANCE;
    }

    /**
     * Sets the power of the intake [%].
     *
     * @param power desired power in percentage.
     */
    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    /**
     * Open intake retractor.
     */
    public void openSolenoid() {
        retractor.set(SolenoidState.OPEN.value);
    }

    /**
     * Close intake retractor.
     */
    public void closeSolenoid() {
        retractor.set(SolenoidState.CLOSED.value);
    }

    /**
     * Toggles intake retractor.
     */
    public void toggleSolenoid() {
        retractor.toggle();
    }

    public enum SolenoidState {
        OPEN(true),
        CLOSED(false);

        public final boolean value;

        SolenoidState(boolean value) {
            this.value = value;
        }
    }
}
