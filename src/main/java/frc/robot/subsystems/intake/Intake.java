package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.utils.Utils;

import static frc.robot.Constants.Intake.IS_COMPENSATING_VOLTAGE;
import static frc.robot.Ports.Hood.SOLENOID;

public class Intake extends SubsystemBase {
    private static Intake INSTANCE;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Intake.MOTOR);
    private final Solenoid retractor;

    private Intake() {
        motor.setInverted(Ports.Intake.IS_MOTOR_INVERTED);
        motor.enableVoltageCompensation(IS_COMPENSATING_VOLTAGE);
        motor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
        if (Utils.isBitSet(Robot.pneumaticsBase.getSolenoids(), SOLENOID)) {
            retractor = new Solenoid(PneumaticsModuleType.CTREPCM, SOLENOID);
        } else {
            retractor = null;
        }
    }


    /**
     * creates an instance of the intake.
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
     * Open intake.
     */
    public void openRetractor() {
        retractor.set(RetractorState.OPEN.value);
    }

    /**
     * Close intake.
     */
    public void closeRetractor() {
        retractor.set(RetractorState.CLOSED.value);
    }

    /**
     * Toggles intake.
     */
    public void toggleRetractor() {
        retractor.toggle();
    }

    public enum RetractorState {
        OPEN(true),
        CLOSED(false);

        public final boolean value;

        RetractorState(boolean value) {
            this.value = value;
        }
    }
}
