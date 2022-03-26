package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.UnitModel;
import frc.robot.utils.Utils;
import webapp.FireLog;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Ports.Shooter.*;

public class Shooter extends SubsystemBase {
    private static Shooter INSTANCE;
    private final UnitModel unitModel = new UnitModel(TICKS_PER_REVOLUTION);
    private final WPI_TalonFX mainMotor = new WPI_TalonFX(MAIN_MOTOR);
    private final WPI_TalonFX auxMotor = new WPI_TalonFX(AUX_MOTOR);
    private final DoubleLogEntry shooterVelocity;
    private final DoubleLogEntry shooterVoltage;

    private Shooter() {
        configureMotor();

        DataLog log = DataLogManager.getLog();
        shooterVelocity = new DoubleLogEntry(log, "/shooter/velocity");
        shooterVoltage = new DoubleLogEntry(log, "/shooter/voltage");

    }

    /**
     * Gets the single instance of the shooter.
     *
     * @return shooter instance.
     */
    public static Shooter getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Shooter();
        }
        return INSTANCE;
    }

    private void configureMotor() {
        mainMotor.configFactoryDefault();
        mainMotor.configAllSettings(getConfiguration());
        mainMotor.setInverted(INVERSION_TYPE);
        mainMotor.setNeutralMode(NeutralMode.Coast);

        var currentLimit = new SupplyCurrentLimitConfiguration(true, 45, 5, 0.1);
        mainMotor.configSupplyCurrentLimit(currentLimit);
        mainMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
        mainMotor.enableVoltageCompensation(true);
        mainMotor.config_IntegralZone(0, 0);

        auxMotor.configFactoryDefault();
        auxMotor.follow(mainMotor);
        auxMotor.configAllSettings(getConfiguration());
        auxMotor.configSupplyCurrentLimit(currentLimit);
        auxMotor.setNeutralMode(NeutralMode.Coast);
        auxMotor.setInverted(TalonFXInvertType.Clockwise);
        auxMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
        auxMotor.enableVoltageCompensation(true);
        auxMotor.config_IntegralZone(0, 0);
    }

    /**
     * Gets the velocity of the motor.
     *
     * @return the velocity of the motor. [rpm]
     */
    public double getVelocity() {
        return Utils.rpsToRpm(unitModel.toVelocity(mainMotor.getSelectedSensorVelocity()));
    }

    /**
     * Sets the velocity of the motor.
     *
     * @param velocity is the velocity setpoint. [rpm]
     */
    public void setVelocity(double velocity) {
        mainMotor.set(ControlMode.Velocity, unitModel.toTicks100ms(Utils.rpmToRps(velocity)));
    }

    /**
     * Set the power for the shooter motor.
     * This should only be used for testing purposes.
     *
     * @param power is the power for the motor. [%]
     */
    public void setPower(double power) {
        mainMotor.set(ControlMode.PercentOutput, power);
    }

    /**
     * Terminates the movement of the wheel.
     */
    public void terminate() {

        mainMotor.stopMotor();
        auxMotor.stopMotor();
    }

    public void changePID() {
        if (mainMotor.isVoltageCompensationEnabled()) {
            mainMotor.enableVoltageCompensation(false);
            auxMotor.enableVoltageCompensation(false);
            kP.set(0.75);
            kI.set(0.00019);
            kD.set(9);
            kF.set(0);
        } else {
            mainMotor.enableVoltageCompensation(true);
            auxMotor.enableVoltageCompensation(true);
            kP.set(0.08);
            kI.set(0.0000075);
            kD.set(9);
            kF.set(0.0465);
        }
    }

    @Override
    public void periodic() {
        FireLog.log("my Shooter velocity", getVelocity());
        FireLog.log("my shooter setpoint", RobotContainer.setpointVelocity);
        shooterVelocity.append(getVelocity());
        shooterVoltage.append(mainMotor.getMotorOutputVoltage());
//        FireLog.log("Shooter-velocity", getVelocity());
//        System.out.println("Shooter'd velocity: " + getVelocity());

        mainMotor.config_kP(0, kP.get());
        mainMotor.config_kI(0, kI.get());
        mainMotor.config_kD(0, kD.get());
        mainMotor.config_kF(0, kF.get());

    }
}