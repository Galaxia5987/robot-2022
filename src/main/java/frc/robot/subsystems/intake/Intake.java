package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;
import webapp.FireLog;

public class Intake extends SubsystemBase {
    private static Intake INSTANCE;
    private final WPI_TalonFX motor = new WPI_TalonFX(Ports.Intake.MOTOR);
    private final Solenoid retractor = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Intake.SOLENOID);
    private final DoubleLogEntry power;
    private final BooleanLogEntry retracted;
    private final UnitModel unitModel = new UnitModel(2048);

    private Intake() {
        motor.setInverted(Ports.Intake.IS_MOTOR_INVERTED);
        motor.enableVoltageCompensation(true);
        motor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE, Constants.TALON_TIMEOUT);
        motor.setNeutralMode(NeutralMode.Brake);
//        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 45, 5, 0.02));
        motor.configClosedloopRamp(0);
        motor.configOpenloopRamp(0);

        DataLog log = DataLogManager.getLog();
        power = new DoubleLogEntry(log, "/intake/power");
        retracted = new BooleanLogEntry(log, "/intake/retracted");
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
     * Gets the power of the intake motor.
     *
     * @return the power of the motor. [%]
     */
    public double getPower() {
        return motor.get();
    }

    /**
     * Sets the power of the intake [%].
     *
     * @param power desired power in percentage.
     */
    public void setPower(double power) {
        FireLog.log("Intake setpoint", power);
        motor.set(power);
    }

    public void setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, unitModel.toTicks100ms(velocity));
    }

    public double getVelocity() {
        return unitModel.toVelocity(motor.getSelectedSensorVelocity());
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

    /**
     * Get the retractors current state.
     */
    public boolean getRetractorState() {
        return retractor.get();
    }

    @Override
    public void periodic() {
        power.append(getPower());
        retracted.append(retractor.get());

//        motor.config_kP(0, Constants.Intake.kP.get(), Constants.TALON_TIMEOUT);
//        motor.config_kI(0, Constants.Intake.kI.get(), Constants.TALON_TIMEOUT);
//        motor.config_kD(0, Constants.Intake.kD.get(), Constants.TALON_TIMEOUT);
//        motor.config_kF(0, Constants.Intake.kF.get(), Constants.TALON_TIMEOUT);
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
