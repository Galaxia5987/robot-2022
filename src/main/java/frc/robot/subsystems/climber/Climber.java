package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;

public class Climber extends SubsystemBase {
    private static Climber INSTANCE = null;
    private final WPI_TalonFX aux = new WPI_TalonFX(Ports.Climber.AUX);
    private final WPI_TalonFX mainMotor = new WPI_TalonFX(Ports.Climber.MAIN);
    private final UnitModel unitModel = new UnitModel(Constants.Climber.TICKS_PER_RAD);

    public Climber() {

        /*
         Set the left motor on Brake mode.
         */
        aux.setNeutralMode(NeutralMode.Brake);

        /*
         sets the phase of the sensor
         */
        aux.setSensorPhase(Ports.Climber.AUX_SENSOR_PHASE);

        /*
         checking is motor inverted.
         */
        aux.setInverted(Ports.Climber.IS_AUX_INVERTED);

        /*
         config PID velocity for left motor.
         */
        aux.config_kP(0, Constants.Climber.P_AUX_VELOCITY);
        aux.config_kI(0, Constants.Climber.I_AUX_VELOCITY);
        aux.config_kD(0, Constants.Climber.D_AUX_VELOCITY);


        /*
         config PID position for left motor.
         */
        aux.config_kP(1, Constants.Climber.P_AUX_POSITION);
        aux.config_kI(1, Constants.Climber.I_AUX_POSITION);
        aux.config_kD(1, Constants.Climber.D_AUX_POSITION);

        /*
         set the right motor on Brake mode.
         */
        mainMotor.setNeutralMode(NeutralMode.Brake);

        /*
        sets the phase of the sensor.
         */
        mainMotor.setSensorPhase(Ports.Climber.MAIN_SENSOR_PHASE);

        /*
         checking is motor inverted.
         */
        mainMotor.setInverted(Ports.Climber.MAIN_SENSOR_PHASE);

        /*
         config PID velocity for right motor.
         */
        mainMotor.config_kP(0, Constants.Climber.P_MAIN_VELOCITY);
        mainMotor.config_kI(0, Constants.Climber.I_MAIN_VELOCITY);
        mainMotor.config_kD(0, Constants.Climber.D_MAIN_VELOCITY);

        /*
         config PID position for right motor.
         */
        mainMotor.config_kP(1, Constants.Climber.P_MAIN_POSITION);
        mainMotor.config_kI(1, Constants.Climber.I_MAIN_POSITION);
        mainMotor.config_kD(1, Constants.Climber.D_MAIN_POSITION);
    }

    /**
     * @return the object Climber.
     */
    public static Climber getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Climber();
        }
        return INSTANCE;
    }

    /**
     * @return get motors velocity. [ticks/rad]
     */
    public double getVelocity() {
        return unitModel.toVelocity(mainMotor.getSelectedSensorVelocity());
    }

    /**
     * @param velocity the velocity of the right & left. [ticks]
     */

    public void setVelocity(double velocity) {
        int ticks100ms = unitModel.toTicks100ms(velocity);
        mainMotor.set(ControlMode.Velocity, ticks100ms);
        aux.set(ControlMode.Velocity, ticks100ms);
    }

    /**
     * @return get motors position. [ticks/rad]
     */
    public double getPosition() {
        return unitModel.toUnits(mainMotor.getSelectedSensorPosition());
    }

    /**
     * @param position the position of the motors. [ticks]
     */
    public void setPosition(double position) {
        mainMotor.set(ControlMode.Position, unitModel.toTicks(position));
    }

    /**
     * set the velocity 0.
     * stop both motors in the place they were.
     */
    public void stop() {
        aux.stopMotor();
        mainMotor.stopMotor();
    }
}