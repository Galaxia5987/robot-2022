package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
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

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Ports.Shooter.INVERSION_TYPE;
import static frc.robot.Ports.Shooter.MOTOR;

public class Shooter extends SubsystemBase {
    private static final ShuffleboardTab tab = Shuffleboard.getTab("Velocity");
    public static NetworkTableEntry shooterVelocity =
            tab.add("Velocity", 0)
                    .getEntry();
    private static Shooter INSTANCE;
    private final UnitModel unitModel = new UnitModel(TICKS_PER_REVOLUTION);
    private final WPI_TalonFX motor = new WPI_TalonFX(MOTOR);
    private FlywheelSim flywheelSim;
    private TalonFXSimCollection simCollection;
    private double currentTime = 0;
    private double lastTime = 0;

    private Shooter() {
        configureMotor();
        if (Robot.isSimulation()) {
            simCollection = motor.getSimCollection();
        }
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
        motor.configAllSettings(getConfiguration());
        motor.setInverted(INVERSION_TYPE);
        motor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
        motor.enableVoltageCompensation(true);
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
        return Utils.rpsToRpm(unitModel.toVelocity(motor.getSelectedSensorVelocity()));
    }

    /**
     * Sets the velocity of the motor.
     *
     * @param velocity is the velocity setpoint. [rpm]
     */
    public void setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, unitModel.toTicks100ms(Utils.rpmToRps(velocity)), DemandType.ArbitraryFeedForward, 0.01);
    }

    /**
     * Set the power for the shooter motor.
     * This should only be used for testing purposes.
     *
     * @param power is the power for the motor. [%]
     */
    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    /**
     * Terminates the movement of the wheel.
     */
    public void terminate() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();

        motor.config_kP(0, kP.get());
        motor.config_kI(0, kI.get());
        motor.config_kD(0, kD.get());
        motor.config_kF(0, kF.get());
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