package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;

public class Climber extends SubsystemBase {
    private static Climber INSTANCE = null;
    private final WPI_TalonFX motorLeft = new WPI_TalonFX(Ports.Climber.MOTOR_LEFT);
    private final WPI_TalonFX motorRight = new WPI_TalonFX(Ports.Climber.MOTOR_RIGHT);
    private final UnitModel unitModelVelocity = new UnitModel(Constants.Climber.VELOCITY_TICKS_PER_UNIT);
    private final UnitModel unitModelDegree = new UnitModel(Constants.Climber.TICKS_PER_DEGREE);

    public Climber() {
        motorLeft.getSelectedSensorPosition();
        motorLeft.neutralOutput();
        motorLeft.config_kP(0, Constants.Climber.P_LEFT_MOTOR);
        motorLeft.config_kP(0, Constants.Climber.I_LEFT_MOTOR);
        motorLeft.config_kP(0, Constants.Climber.D_LEFT_MOTOR);

        motorRight.getSelectedSensorPosition();
        motorRight.neutralOutput();
        motorRight.config_kP(0, Constants.Climber.P_RIGHT_MOTOR);
        motorRight.config_kP(0, Constants.Climber.I_RIGHT_MOTOR);
        motorRight.config_kP(0, Constants.Climber.D_RIGHT_MOTOR);
    }

    public static Climber getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Climber();
        }
        return INSTANCE;
    }

    public void setVelocity(double velocity) {
        motorRight.set(ControlMode.Velocity, unitModelVelocity.toTicks100ms(velocity));
        motorLeft.set(ControlMode.Velocity, unitModelVelocity.toTicks100ms(velocity));
    }

    public double getVelocityRight() {
        return unitModelVelocity.toVelocity(motorRight.getSelectedSensorVelocity());
    }

    public double getVelocityLeft() {
        return unitModelVelocity.toVelocity(motorLeft.getSelectedSensorVelocity());
    }

    public void setPositionRight(double rightPosition) {
        motorRight.set(ControlMode.Position, unitModelDegree.toTicks100ms(rightPosition));
    }

    public void setPositionLeft(double leftPosition) {
        motorLeft.set(ControlMode.Position, unitModelDegree.toTicks100ms(leftPosition));
    }

    public double getPositionRight() {
        return unitModelDegree.toUnits(motorRight.getSelectedSensorPosition());
    }

    public double getPositionLeft() {
        return unitModelDegree.toUnits(motorLeft.getSelectedSensorPosition());
    }

    public void stop() {
        setVelocity(0);
        setPositionLeft(getPositionLeft());
        setPositionRight(getPositionRight());
    }
}
