package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.subsystems.UnitModel;

public class Climber extends SubsystemBase {
    private static Climber INSTANCE = null;
    private final WPI_TalonFX leftMotor = new WPI_TalonFX(Ports.Climber.LEFT);
    private final WPI_TalonFX rightMotor = new WPI_TalonFX(Ports.Climber.RIGHT);
    private final UnitModel unitModel = new UnitModel(Constants.Climber.TICKS_PER_RAD);
    private EncoderSim encoderSim;
    private MechanismLigament2d m_line;
    private MechanismLigament2d m_wrist;


    public Climber() {

        /*
         Set the left motor on Brake mode.
         */
        leftMotor.setNeutralMode(NeutralMode.Brake);

        /*
         sets the phase of the sensor
         */
        leftMotor.setSensorPhase(Ports.Climber.LEFT_SENSOR_PHASE);

        /*
         checking is motor inverted.
         */
        leftMotor.setInverted(Ports.Climber.IS_LEFT_INVERTED);

        /*
         config PID velocity for left motor.
         */
        leftMotor.config_kP(0, Constants.Climber.P_VELOCITY);
        leftMotor.config_kI(0, Constants.Climber.I_VELOCITY);
        leftMotor.config_kD(0, Constants.Climber.D_VELOCITY);


        /*
         config PID position for left motor.
         */
        leftMotor.config_kP(1, Constants.Climber.P_POSITION);
        leftMotor.config_kI(1, Constants.Climber.I_POSITION);
        leftMotor.config_kD(1, Constants.Climber.D_POSITION);

        /*
         set the right motor on Brake mode.
         */
        rightMotor.setNeutralMode(NeutralMode.Brake);

        /*
        sets the phase of the sensor
         */
        rightMotor.setSensorPhase(Ports.Climber.RIGHT_SENSOR_PHASE);

        /*
         checking is motor inverted.
         */
        rightMotor.setInverted(Ports.Climber.IS_RIGHT_INVERTED);

        /*
         config PID velocity for right motor.
         */
        rightMotor.config_kP(0, Constants.Climber.P_VELOCITY);
        rightMotor.config_kI(0, Constants.Climber.I_VELOCITY);
        rightMotor.config_kD(0, Constants.Climber.D_VELOCITY);

        /*
         config PID position for right motor.
         */
        rightMotor.config_kP(1, Constants.Climber.P_POSITION);
        rightMotor.config_kI(1, Constants.Climber.I_POSITION);
        rightMotor.config_kD(1, Constants.Climber.D_POSITION);

        if (Robot.isSimulation()) {
            Encoder encoder = new Encoder(0, 1);
            encoder.setDistancePerPulse(2 * Math.PI);
            encoderSim = new EncoderSim(encoder);
            Mechanism2d mech = new Mechanism2d(3, 3);
            // the mechanism root node
            MechanismRoot2d root = mech.getRoot("climber", 2, 0);

            // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
            // off the root node or another ligament object
            m_line = root.append(new MechanismLigament2d("elevator", 1, 90));
            m_wrist =
                    m_line.append(
                            new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));

            // post the mechanism to the dashboard
            SmartDashboard.putData("Mech2d", mech);
        }
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

    // TODO: add units
    /**
     * @return get the right motor velocity.
     */
    public double getRightVelocity() {
        return unitModel.toVelocity(rightMotor.getSelectedSensorVelocity());
    }

    // TODO: add units
    /**
     * @return get the left motor velocity.
     */
    public double getLeftVelocity() {
        if (Robot.isSimulation()) {
            return encoderSim.getRate();
        }
        return unitModel.toVelocity(leftMotor.getSelectedSensorVelocity());
    }

    // TODO: add units
    /**
     * @param velocity the velocity of the right & left.
     */
    public void setVelocity(double velocity) {
        if (Robot.isSimulation()) {
            encoderSim.setRate(velocity);
        }
        int tick100ms = unitModel.toTicks100ms(velocity);
        rightMotor.set(ControlMode.Velocity, tick100ms);
        leftMotor.set(ControlMode.Velocity, tick100ms);
    }

    // TODO: add units
    /**
     * @return get the right motor position.
     */
    public double getPositionRight() {
        return unitModel.toUnits(rightMotor.getSelectedSensorPosition());
    }

    // TODO: add units
    /**
     * @param rightPosition the position of the right.
     */
    public void setPositionRight(double rightPosition) {
        rightMotor.set(ControlMode.Position, unitModel.toTicks(rightPosition));
    }

    // TODO: add units
    /**
     * @return get the left motor position.
     */
    public double getPositionLeft() {
        return unitModel.toUnits(leftMotor.getSelectedSensorPosition());
    }

    // TODO: add units
    /**
     * @param leftPosition the position of the left.
     */
    public void setPositionLeft(double leftPosition) {
        leftMotor.set(ControlMode.Position, unitModel.toTicks(leftPosition));
    }

    /**
     * set the velocity 0.
     * stop both motors in the place they were.
     */
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putNumber("Jdam", getLeftVelocity());
        m_wrist.setAngle(m_wrist.getAngle() + getLeftVelocity() * 0.02);
    }
}