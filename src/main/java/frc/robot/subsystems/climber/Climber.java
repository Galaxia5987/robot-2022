package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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

    private final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
    private final PIDController m_controller = new PIDController(1.5, 0, 0);

    private final Encoder m_encoder = new Encoder(Ports.Climber.ENCODER_A_CHANNEL, Ports.Climber.ENCODER_B_CHANNEL);
    private final DCMotor m_armGearbox = DCMotor.getFalcon500(2);

    private final SingleJointedArmSim m_armSim =
            new SingleJointedArmSim(
                    m_armGearbox,
                    Constants.Climber.GEAR_RATIO,
                    SingleJointedArmSim.estimateMOI(Constants.Climber.ARM_LENGTH, Constants.Climber.ARM_MASS),
                    Constants.Climber.ARM_LENGTH,
                    Units.degreesToRadians(-75),
                    Units.degreesToRadians(255),
                    Constants.Climber.ARM_MASS,
                    true,
                    VecBuilder.fill(Constants.Climber.ARM_ENCODER_DIST_PER_PULSE)
            );

    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d m_armTower =
            m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));

    private final MechanismLigament2d m_arm =
            m_armPivot.append(
                    new MechanismLigament2d(
                            "Arm",
                            30,
                            Units.radiansToDegrees(m_armSim.getAngleRads()),
                            6,
                            new Color8Bit(Color.kYellow)));


    public Climber() {
        if (Robot.isSimulation()) {
            SmartDashboard.putData("Arm sim", m_mech2d);
            m_encoder.setDistancePerPulse(Constants.Climber.ARM_ENCODER_DIST_PER_PULSE);
            m_armTower.setColor(new Color8Bit(Color.kBlue));
        }


        leftMotor.follow(rightMotor);

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
        sets the phase of the sensor.
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
     * @return get motors velocity.
     */
    public double getVelocity() {
        if (Robot.isSimulation()) {
            return m_encoderSim.getRate();
        }
        return unitModel.toVelocity(leftMotor.getSelectedSensorVelocity());
    }

    // TODO: add units

    /**
     * @param velocity the velocity of the right & left.
     */

    public void setVelocity(double velocity) {
        if (Robot.isSimulation()) {
            double volts = m_controller.calculate(getVelocity(), velocity);
            rightMotor.setVoltage(volts + feedforward.calculate(0, 0));
        } else {
            int tick100ms = unitModel.toTicks100ms(velocity);
            rightMotor.set(ControlMode.Velocity, tick100ms);
        }
    }

    // TODO: add units

    /**
     * @return get motors position.
     */
    public double getPosition() {
        return unitModel.toUnits(rightMotor.getSelectedSensorPosition());
    }

    // TODO: add units

    /**
     * @param position the position of the motors.
     */
    public void setPosition(double position) {
        rightMotor.set(ControlMode.Position, unitModel.toTicks(position));
    }

    // TODO: add units

    /**
     * stop both motors in the place they were.
     */
    public void stop() {
        rightMotor.stopMotor();
    }

    /**
     * Add periodic for the simulation.
     */
    @Override
    public void simulationPeriodic() {
        m_armSim.setInput(rightMotor.get() * RobotController.getBatteryVoltage());

        m_armSim.update(0.02);

        m_encoderSim.setDistance(m_armSim.getAngleRads());
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
        m_encoderSim.setRate(m_armSim.getVelocityRadPerSec());
        m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    }
}

