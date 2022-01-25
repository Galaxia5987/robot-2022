package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
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

    private final WPI_TalonFX aux = new WPI_TalonFX(Ports.Climber.AUX);
    private final WPI_TalonFX mainMotor = new WPI_TalonFX(Ports.Climber.MAIN_MOTOR);
    private final Solenoid stopper = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Climber.STOPPER);

    private final UnitModel unitModel = new UnitModel(Constants.Climber.TICKS_PER_RAD);

    private final ArmFeedforward feedforward = new ArmFeedforward(Constants.Climber.F_FORWARD_S, Constants.Climber.F_FORWARD_COS, Constants.Climber.F_FORWARD_V, Constants.Climber.F_FORWARD_A);

    private final PIDController m_controller = new PIDController(Constants.Climber.P_VELOCITY, Constants.Climber.I_VELOCITY, Constants.Climber.D_VELOCITY);

    private final Encoder m_encoder = new Encoder(Ports.Climber.ENCODER_A_CHANNEL, Ports.Climber.ENCODER_B_CHANNEL);
    private final DCMotor m_armGearbox = DCMotor.getFalcon500(2);

    private final SingleJointedArmSim m_armSim =
            new SingleJointedArmSim(
                    m_armGearbox,
                    Constants.Climber.GEAR_RATIO,
                    SingleJointedArmSim.estimateMOI(Constants.Climber.ARM_LENGTH, Constants.Climber.ARM_MASS),
                    Constants.Climber.ARM_LENGTH,
                    Units.degreesToRadians(Constants.Climber.MIN_ANGLE),
                    Units.degreesToRadians(Constants.Climber.MAX_ANGLE),
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
        mainMotor.setInverted(Ports.Climber.IS_MAIN_INVERTED);

        /*
         config PID velocity for main motor.
         */
        mainMotor.configMotionCruiseVelocity(getVelocity());
        mainMotor.config_kP(0, Constants.Climber.P_VELOCITY);
        mainMotor.config_kI(0, Constants.Climber.I_VELOCITY);
        mainMotor.config_kD(0, Constants.Climber.D_VELOCITY);

        aux.follow(mainMotor);

        /*
         Set the aux motor on Brake mode.
         */
        aux.setNeutralMode(NeutralMode.Brake);

        /*
         sets the phase of the sensor.
         */
        aux.setSensorPhase(Ports.Climber.AUX_SENSOR_PHASE);

        /*
         checking is aux inverted.
         */
        aux.setInverted(Ports.Climber.IS_AUX_INVERTED);

        /*
         config PID velocity for aux motor.
         */
        aux.configMotionCruiseVelocity(getVelocity());
        aux.config_kP(0, Constants.Climber.P_VELOCITY);
        aux.config_kI(0, Constants.Climber.I_VELOCITY);
        aux.config_kD(0, Constants.Climber.D_VELOCITY);
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
        if (Robot.isSimulation()) {
            return m_encoderSim.getRate();
        }
        return unitModel.toVelocity(mainMotor.getSelectedSensorVelocity());
    }

    /**
     * @param velocity the velocity of the motors. [ticks]
     */

    public void setVelocity(double velocity) {
        if (Robot.isSimulation()) {
            double volts = m_controller.calculate(getVelocity(), velocity);
            mainMotor.setVoltage(volts + feedforward.calculate(0, 0));
        } else {
            int tick100ms = unitModel.toTicks100ms(velocity);
            mainMotor.set(ControlMode.Velocity, tick100ms);
        }
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
        mainMotor.set(ControlMode.MotionMagic, unitModel.toTicks(position));
    }

    public boolean isStopperEngaged() {
        return stopper.get();
    }

    public void setStopperMode(boolean engaged) {
        stopper.set(!engaged);
    }

    public void toggleStopper() {
        setStopperMode(!isStopperEngaged());
    }

    /**
     * stop both motors in the place they were.
     */
    public void stop() {
        mainMotor.stopMotor();
    }


    /**
     * Add periodic for the simulation.
     */
    @Override
    public void simulationPeriodic() {
        m_armSim.setInput(mainMotor.get() * RobotController.getBatteryVoltage());

        m_armSim.update(0.02);

        m_encoderSim.setDistance(m_armSim.getAngleRads());
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
        m_encoderSim.setRate(m_armSim.getVelocityRadPerSec());
        m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    }
}