package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;

public class Climber extends SubsystemBase {
    private WPI_TalonFX motorLeft = new WPI_TalonFX(Ports.Climber.MOTOR_LEFT);
    private WPI_TalonFX motorRight = new WPI_TalonFX(Ports.Climber.MOTOR_RIGHT);
    private UnitModel unitModel = new UnitModel(Constants.Climber.UNIT_MODEL);

    public Climber() {
        motorLeft.config_kP(0, Constants.Climber.P_LEFT_MOTOR);
        motorLeft.config_kP(0, Constants.Climber.I_LEFT_MOTOR);
        motorLeft.config_kP(0, Constants.Climber.D_LEFT_MOTOR);

        motorLeft.config_kP(0, Constants.Climber.P_RIGHT_MOTOR);
        motorLeft.config_kP(0, Constants.Climber.I_RIGHT_MOTOR);
        motorLeft.config_kP(0, Constants.Climber.D_RIGHT_MOTOR);

    }

    public void setVelocity(double velocity){

    }

    public static double getVelocity(){
        return
    }

    public void stop(){
        setVelocity(0);
    }

    public static double getPosition(){
        return
    }

}

