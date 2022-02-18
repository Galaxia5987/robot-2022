package frc.robot.subsystems.helicopter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
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


public class Helicopter extends SubsystemBase {

    private static Helicopter INSTANCE = null;

    private final WPI_TalonFX mainMotor = new WPI_TalonFX(Ports.Helicopter.MAIN);
    private final WPI_TalonFX auxMotor = new WPI_TalonFX(Ports.Helicopter.AUX);
    private final Solenoid stopper = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Helicopter.STOPPER);
    private final UnitModel unitModelPosition = new UnitModel(Constants.Helicopter.TICKS_PER_RAD);


    private final PIDController controller = new PIDController(Constants.Helicopter.KP, Constants.Helicopter.KI, Constants.Helicopter.KD);
    private final ArmFeedforward feedforward = new ArmFeedforward(Constants.Helicopter.F_FORWARD_S, Constants.Helicopter.F_FORWARD_COS, Constants.Helicopter.F_FORWARD_V, Constants.Helicopter.F_FORWARD_A);
    private final Encoder encoder = new Encoder(Ports.Helicopter.ENCODER_A_CHANNEL, Ports.Helicopter.ENCODER_B_CHANNEL);
    private final DCMotor armGearbox = DCMotor.getFalcon500(2);


    private final SingleJointedArmSim armSim =
            new SingleJointedArmSim(
                    armGearbox,
                    Constants.Helicopter.GEAR_RATIO,
                    SingleJointedArmSim.estimateMOI(Constants.Helicopter.ARM_LENGTH, Constants.Helicopter.ARM_MASS),
                    Constants.Helicopter.ARM_LENGTH,
                    Constants.Helicopter.MIN_ANGLE,
                    Constants.Helicopter.MAX_ANGLE,
                    Constants.Helicopter.ARM_MASS,
                    true,
                    VecBuilder.fill(Constants.Helicopter.ARM_ENCODER_DIST_PER_PULSE)
            );

    private final EncoderSim encoderSim = new EncoderSim(encoder);

    private final Mechanism2d mechanism2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d armPivot = mechanism2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d armTower =
            armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));

    private final MechanismLigament2d arm =
            armPivot.append(
                    new MechanismLigament2d(
                            "Arm",
                            30,
                            Units.radiansToDegrees(armSim.getAngleRads()),
                            6,
                            new Color8Bit(Color.kYellow)));


    private Helicopter() {
        if (Robot.isSimulation()) {
            SmartDashboard.putData("Arm sim", mechanism2d);
            encoder.setDistancePerPulse(Constants.Helicopter.ARM_ENCODER_DIST_PER_PULSE);
            armTower.setColor(new Color8Bit(Color.kBlue));
        }

        mainMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 1, Constants.TALON_TIMEOUT);

        /*
         Set sensor phase.
         */
        mainMotor.setSensorPhase(Ports.Helicopter.SENSOR_PHASE);
        /*
         Set the right motor on Brake mode.
         */
        mainMotor.setNeutralMode(NeutralMode.Brake);

        /*
         Setting the motor to go clockwise.
         */
        mainMotor.setInverted(Ports.Helicopter.IS_MAIN_INVERTED);

        /*
         config PID velocity for main motor.
         */
        mainMotor.configMotionCruiseVelocity(Constants.Helicopter.CRUISE_VELOCITY);
        mainMotor.configMotionAcceleration(Constants.Helicopter.MAXIMAL_ACCELERATION);
        mainMotor.config_kP(0, Constants.Helicopter.KP, Constants.TALON_TIMEOUT);
        mainMotor.config_kI(0, Constants.Helicopter.KI, Constants.TALON_TIMEOUT);
        mainMotor.config_kD(0, Constants.Helicopter.KD, Constants.TALON_TIMEOUT);


        auxMotor.follow(mainMotor);

        mainMotor.enableVoltageCompensation(Constants.Helicopter.VOLTAGE_COMPENSATION);

        mainMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);

        auxMotor.enableVoltageCompensation(Constants.Helicopter.VOLTAGE_COMPENSATION);

        auxMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);

        /*
         Set the aux motor on Brake mode.
         */
        auxMotor.setNeutralMode(NeutralMode.Brake);

        /*
         Setting the motor to go clockwise.
         */
        auxMotor.setInverted(Ports.Helicopter.IS_AUX_INVERTED);
    }

    /**
     * @return the object Helicpter.
     */
    public static Helicopter getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Helicopter();
        }
        return INSTANCE;
    }

    /**
     * @return get motor velocity. [rad/s]
     */
    public double getVelocity() {
        if (Robot.isSimulation()) {
            return encoderSim.getRate();
        }
        return unitModelPosition.toVelocity(mainMotor.getSelectedSensorVelocity(0));
    }

    /**
     * @param velocity the velocity of the motors. [rad/s]
     */
    public void setVelocity(double velocity) {
        if (Robot.isSimulation()) {
            double volts = controller.calculate(getVelocity(), velocity);
            mainMotor.setVoltage(volts + feedforward.calculate(0, 0));
        } else {
            mainMotor.set(ControlMode.Velocity, unitModelPosition.toTicks100ms(velocity));
        }
    }

    /**
     * Set Helicpter position to zero.
     * Zero is the balanced position of the arms.
     */
    public void setAngleZero() {
        double angle = getAbsolutePosition();
        setPosition(getPosition() - angle);
    }

    /**
     * @return the absolute position of the Helicpter.
     */
    public double getAbsolutePosition() {
        return unitModelPosition.toUnits(mainMotor.getSelectedSensorPosition(1) - Constants.Helicopter.ZERO_POSITION);
    }


    /**
     * @return get motors position. [rad]
     */
    public double getPosition() {
        return unitModelPosition.toUnits(mainMotor.getSelectedSensorPosition(0));
    }

    /**
     * @param position the position of the motors. [rad]
     */
    public void setPosition(double position) {
        mainMotor.set(ControlMode.MotionMagic, unitModelPosition.toTicks(position),
                DemandType.ArbitraryFeedForward, feedforward.calculate(getPosition(), getVelocity()));
    }

    /**
     * Get the stopper's mode.
     *
     * @return the stopper's mode.
     */
    public boolean isStopperEngaged() {
        return stopper.get();
    }

    /**
     * Set the stopper's mode.
     *
     * @param engaged whether the stopper is engaged.
     */
    public void setStopperMode(boolean engaged) {
        stopper.set(!engaged);
    }

    /**
     * Toggle the value of the stopper.
     */
    public void toggleStopper() {
        stopper.toggle();
    }

    /**
     * Stop both motors in the place they were.
     */
    public void stop() {
        mainMotor.stopMotor();
    }

    /**
     * Add periodic for the simulation.
     */
    @Override
    public void simulationPeriodic() {
        armSim.setInput(mainMotor.get() * RobotController.getBatteryVoltage());

        armSim.update(Constants.SIMULATION_LOOP_PERIOD);

        encoderSim.setDistance(armSim.getAngleRads());
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
        encoderSim.setRate(armSim.getVelocityRadPerSec());
        arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    }
}