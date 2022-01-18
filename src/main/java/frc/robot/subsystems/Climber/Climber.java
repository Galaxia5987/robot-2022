package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;

public class Climber extends SubsystemBase {
    private static Climber INSTANCE = null;
    private final WPI_TalonFX motorLeft = new WPI_TalonFX(Ports.Climber.LEFT);
    private final WPI_TalonFX motorRight = new WPI_TalonFX(Ports.Climber.RIGHT);
    private final UnitModel unitModelVelocity = new UnitModel(Constants.Climber.TICKS_PER_METER);
    private final UnitModel unitModelDegree = new UnitModel(Constants.Climber.TICKS_PER_DEGREE);

    public Climber() {
        /*
         Set the left motor on Brake mode.
         */
        motorLeft.setNeutralMode(NeutralMode.Brake);

        /*
         sets the phase of the sensor
         */
        motorLeft.setSensorPhase(Constants.Climber.SENSOR_PHASE);

        /*
         checking is motor inverted.
         */
        motorLeft.setInverted(Constants.Climber.IS_INVERTED);

        /*
         config PID velocity for left motor.
         */
        motorLeft.config_kP(0, Constants.Climber.P_LEFT_VELOCITY);
        motorLeft.config_kI(0, Constants.Climber.I_LEFT_VELOCITY);
        motorLeft.config_kD(0, Constants.Climber.D_LEFT_VELOCITY);

        /*
         config PID position for left motor.
         */
        motorRight.config_kP(0, Constants.Climber.P_LEFT_POSITION);
        motorRight.config_kI(0, Constants.Climber.I_LEFT_POSITION);
        motorRight.config_kD(0, Constants.Climber.D_LEFT_POSITION);

        /*
         set the right motor on Brake mode.
         */
        motorRight.setNeutralMode(NeutralMode.Brake);

        /*
        sets the phase of the sensor
         */
        motorLeft.setSensorPhase(Constants.Climber.SENSOR_PHASE);

        /*
         checking is motor inverted.
         */
        motorLeft.setInverted(Constants.Climber.IS_INVERTED);

        /*
         config PID velocity for right motor.
         */
        motorRight.config_kP(0, Constants.Climber.P_RIGHT_VELOCITY);
        motorRight.config_kI(0, Constants.Climber.I_RIGHT_VELOCITY);
        motorRight.config_kD(0, Constants.Climber.D_RIGHT_VELOCITY);

        /*
         config PID position for right motor.
         */
        motorRight.config_kP(0, Constants.Climber.P_RIGHT_POSITION);
        motorRight.config_kI(0, Constants.Climber.I_RIGHT_POSITION);
        motorRight.config_kD(0, Constants.Climber.D_RIGHT_POSITION);
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
     * @return get the right motor velocity.
     */
    public double getVelocityRight() {
        return unitModelVelocity.toVelocity(motorRight.getSelectedSensorVelocity());
    }

    /**
     * @return get the left motor velocity.
     */
    public double getVelocityLeft() {
        return unitModelVelocity.toVelocity(motorLeft.getSelectedSensorVelocity());
    }

    /**
     * @param velocity the velocity of the right & left.
     */
    public void setVelocity(double velocity) {
        motorRight.set(ControlMode.Velocity, unitModelVelocity.toTicks100ms(velocity));
        motorLeft.set(ControlMode.Velocity, unitModelVelocity.toTicks100ms(velocity));
    }

    /**
     * @return get the right motor position.
     */
    public double getPositionRight() {
        return unitModelDegree.toUnits(motorRight.getSelectedSensorPosition());
    }

    /**
     * @param rightPosition the position of the right.
     */
    public void setPositionRight(double rightPosition) {
        motorRight.set(ControlMode.Position, unitModelDegree.toTicks(rightPosition));
    }

    /**
     * @return get the left motor position.
     */
    public double getPositionLeft() {
        return unitModelDegree.toUnits(motorLeft.getSelectedSensorPosition());
    }

    /**
     * @param leftPosition the position of the left.
     */
    public void setPositionLeft(double leftPosition) {
        motorLeft.set(ControlMode.Position, unitModelDegree.toTicks(leftPosition));
    }

    /**
     * set the velocity 0.
     * stop both motors in the place they were.
     */
    public void stop() {
        setVelocity(0);
        setPositionLeft(getPositionLeft());
        setPositionRight(getPositionRight());
    }
}