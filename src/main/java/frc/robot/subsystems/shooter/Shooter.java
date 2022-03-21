package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.UnitModel;
import frc.robot.utils.Utils;
import webapp.FireLog;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Ports.Shooter.*;

public class Shooter extends SubsystemBase {
    private static final ShuffleboardTab tab = Shuffleboard.getTab("Velocity");
    private static Shooter INSTANCE;
    private final UnitModel unitModel = new UnitModel(TICKS_PER_REVOLUTION);
    private final WPI_TalonFX mainMotor = new WPI_TalonFX(MAIN_MOTOR);
    private final WPI_TalonFX auxMotor = new WPI_TalonFX(AUX_MOTOR);
    private final DoubleLogEntry shooterVelocity;
    private final DoubleLogEntry shooterVoltage;
    private FlywheelSim flywheelSim;
    private TalonFXSimCollection simCollection;
    private double currentTime = 0;
    private double lastTime = 0;

    private Shooter() {
        configureMotor();
        if (Robot.isSimulation()) {
            simCollection = mainMotor.getSimCollection();
        }

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
        mainMotor.configAllSettings(getConfiguration());
        mainMotor.setInverted(INVERSION_TYPE);
//        mainMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
        mainMotor.enableVoltageCompensation(false);

        auxMotor.follow(mainMotor);
        auxMotor.configAllSettings(getConfiguration());
        auxMotor.setInverted(TalonFXInvertType.OpposeMaster);
//        auxMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
        auxMotor.enableVoltageCompensation(false);
    }

    /**
     * Gets the velocity of the motor.
     *
     * @return the velocity of the motor. [rpm]
     */
    public double getVelocity() {
        if (Robot.isSimulation()) {
            return flywheelSim.getAngularVelocityRPM();
        }
        return Utils.rpsToRpm(unitModel.toVelocity(mainMotor.getSelectedSensorVelocity()));
    }

    /**
     * Sets the velocity of the motor.
     *
     * @param velocity is the velocity setpoint. [rpm]
     */
    public void setVelocity(double velocity) {
        mainMotor.set(ControlMode.Velocity, unitModel.toTicks100ms(Utils.rpmToRps(velocity)), DemandType.ArbitraryFeedForward, 0.01);
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
    }

    @Override
    public void periodic() {
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
        shooterVelocity.append(getVelocity());
        shooterVoltage.append(mainMotor.getMotorOutputVoltage());
        FireLog.log("Shooter-velocity", getVelocity());

        mainMotor.config_kP(0, kP.get());
        mainMotor.config_kI(0, kI.get());
//        mainMotor.configMaxIntegralAccumulator(0, maxIntegralAccumulation.get());
        FireLog.log("accI", mainMotor.getIntegralAccumulator());
        System.out.println(mainMotor.getIntegralAccumulator());
        mainMotor.config_kD(0, kD.get());
        mainMotor.config_kF(0, kF.get());
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putNumber("velocity", flywheelSim.getAngularVelocityRPM());
        SmartDashboard.putNumber("power", simCollection.getMotorOutputLeadVoltage());
        flywheelSim.update(currentTime - lastTime);
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
    }
}