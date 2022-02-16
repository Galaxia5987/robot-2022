package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.simulation.BatterySim;
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

    private final WPI_TalonFX mainMotor = new WPI_TalonFX(Ports.Climber.MAIN);
    private final WPI_TalonFX auxMotor = new WPI_TalonFX(Ports.Climber.AUX);
    private final Solenoid stopper = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Climber.STOPPER);
    private final UnitModel unitModelPosition = new UnitModel(Constants.Climber.TICKS_PER_RAD);
    private final DutyCycleEncoder dutyCycleEncoder = new DutyCycleEncoder(Ports.Climber.ENCODER);

    private final PIDController controller;
    private final SingleJointedArmSim armSim;
    private final ArmFeedforward feedforward;
    private final DCMotor armGearbox;
    private final MechanismLigament2d arm;


    private Climber() {
        if (Robot.isSimulation()) {

            controller = new PIDController(Constants.Climber.KP, Constants.Climber.KI, Constants.Climber.KD);
            feedforward = new ArmFeedforward(Constants.Climber.F_FORWARD_S, Constants.Climber.F_FORWARD_COS, Constants.Climber.F_FORWARD_V, Constants.Climber.F_FORWARD_A);
            armGearbox = DCMotor.getFalcon500(2);


            armSim =
                    new SingleJointedArmSim(
                            armGearbox,
                            Constants.Climber.GEAR_RATIO,
                            SingleJointedArmSim.estimateMOI(Constants.Climber.ARM_LENGTH, Constants.Climber.ARM_MASS),
                            Constants.Climber.ARM_LENGTH,
                            Constants.Climber.MIN_ANGLE,
                            Constants.Climber.MAX_ANGLE,
                            Constants.Climber.ARM_MASS,
                            true,
                            VecBuilder.fill(Constants.Climber.ARM_ENCODER_DIST_PER_PULSE)
                    );


            Mechanism2d mechanism2d = new Mechanism2d(60, 60);
            MechanismRoot2d armPivot = mechanism2d.getRoot("ArmPivot", 30, 30);
            MechanismLigament2d armTower =
                    armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));

             arm =
                    armPivot.append(
                            new MechanismLigament2d(
                                    "Arm",
                                    30,
                                    Units.radiansToDegrees(armSim.getAngleRads()),
                                    6,
                                    new Color8Bit(Color.kYellow)));

            SmartDashboard.putData("Arm sim", mechanism2d);
            armTower.setColor(new Color8Bit(Color.kBlue));
        } else {
            controller = null;
            armSim = null;
            feedforward = null;
            armGearbox = null;
            arm = null;
        }

        /*
         Set sensor phase.
         */
        mainMotor.setSensorPhase(Ports.Climber.SENSOR_PHASE);
        /*
         Set the right motor on Brake mode.
         */
        mainMotor.setNeutralMode(NeutralMode.Brake);

        /*
         Setting the motor to go clockwise.
         */
        mainMotor.setInverted(Ports.Climber.IS_MAIN_INVERTED);

        /*
         config PID velocity for main motor.
         */
        mainMotor.configMotionCruiseVelocity(Constants.Climber.CRUISE_VELOCITY);
        mainMotor.configMotionAcceleration(Constants.Climber.MAXIMAL_ACCELERATION);
        mainMotor.config_kP(0, Constants.Climber.KP, Constants.TALON_TIMEOUT);
        mainMotor.config_kI(0, Constants.Climber.KI, Constants.TALON_TIMEOUT);
        mainMotor.config_kD(0, Constants.Climber.KD, Constants.TALON_TIMEOUT);


        auxMotor.follow(mainMotor);

        mainMotor.enableVoltageCompensation(Constants.Climber.VOLTAGE_COMPENSATION);

        mainMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);

        auxMotor.enableVoltageCompensation(Constants.Climber.VOLTAGE_COMPENSATION);

        auxMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);

        /*
         Set the aux motor on Brake mode.
         */
        auxMotor.setNeutralMode(NeutralMode.Brake);

        /*
         Setting the motor to go clockwise.
         */
        auxMotor.setInverted(Ports.Climber.IS_AUX_INVERTED);
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
     * @return get motor velocity. [rad/s]
     */
    public double getVelocity() {
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
     * Set climber position to zero.
     * Zero is the balanced position of the arms.
     */
    public void setAngleZero() {
        double angle = getAbsolutePosition();
        setPosition(getPosition() - angle);
    }

    /**
     * @return the absolute position of the Climber.
     */
    public double getAbsolutePosition() {
        return unitModelPosition.toUnits(dutyCycleEncoder.getDistance() - Constants.Climber.ZERO_POSITION);
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

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
        mainMotor.getSimCollection().setIntegratedSensorRawPosition(unitModelPosition.toTicks(armSim.getAngleRads()));
        arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    }
}