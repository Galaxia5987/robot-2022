package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;

import java.util.ArrayDeque;
import java.util.Deque;

import static frc.robot.Constants.Conveyor.*;
import static frc.robot.Constants.TALON_TIMEOUT;
import static frc.robot.Ports.Conveyor.IS_COMPENSATING_VOLTAGE;
import static frc.robot.Ports.Conveyor.MOTOR_INVERSION;

public class Conveyor extends SubsystemBase {
    private static Conveyor INSTANCE = null;
    private final WPI_TalonFX motor = new WPI_TalonFX(Ports.Conveyor.MOTOR);
    private final Deque<String> cargoPositions = new ArrayDeque<>();
    private final DigitalInput postFlapBeam = new DigitalInput(Ports.Conveyor.POST_FLAP_BEAM_BREAKER);
    private final DigitalInput preFlapBeam = new DigitalInput(Ports.Conveyor.PRE_FLAP_BEAM_BREAKER);
    private final UnitModel unitModel = new UnitModel(Constants.Conveyor.TICKS_PER_UNIT);
    private final boolean wasPostFlapBeamConnected = true;
    private final int currentProximity = 0;
    private double commandPower;
    private final LinearFilter filter = LinearFilter.movingAverage(20);
    private final ColorSensor colorSensor = new ColorSensor(I2C.Port.kMXP);

    private Conveyor() {
        motor.setInverted(MOTOR_INVERSION);
        motor.enableVoltageCompensation(IS_COMPENSATING_VOLTAGE);
        motor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
        cargoPositions.add(DriverStation.Alliance.Invalid.name());
        cargoPositions.add(DriverStation.Alliance.Invalid.name());
        motor.config_kP(0, kP, TALON_TIMEOUT);
        motor.config_kI(0, kI, TALON_TIMEOUT);
        motor.config_kD(0, kD, TALON_TIMEOUT);
        motor.config_kF(0, kF, TALON_TIMEOUT);
    }

    /**
     * lazy instantiation
     *
     * @return the subsystem instance
     */
    public static Conveyor getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Conveyor();
        }
        return INSTANCE;
    }

    /**
     * Get the input of the pre-flap beam breaker.
     *
     * @return the input of the pre-flap beam breaker (true or false).
     */
    public boolean isPreFlapBeamConnected() {
        return preFlapBeam.get();
    }

    /**
     * Get input of the post-flap beam breaker.
     *
     * @return the input of the post-flap beam breaker (true or false).
     */
    public boolean isPostFlapBeamConnected() {
        return postFlapBeam.get();
    }

    /**
     * @return the amount of balls inside the cargo.
     */
    public int getCargoCount() {
        int count = 0;
        if (!cargoPositions.getFirst().equals(DriverStation.Alliance.Invalid.name())) count++;
        if (!cargoPositions.getLast().equals(DriverStation.Alliance.Invalid.name())) count++;
        return count;
    }

    /**
     * Gets the power of the motor.
     *
     * @return the power of the motor. [%]
     */
    public double getPower() {
        return motor.get();
    }

    /**
     * Set motor power.
     *
     * @param power the power to the motor [-1, 1].
     */
    public void setPower(double power) {
        motor.set(power);
    }

    public void setCommandPower(double commandPower) {
        this.commandPower = commandPower;
    }

    public double getVelocity() {
        return unitModel.toVelocity(motor.getSelectedSensorVelocity()) * 60;
    }

    public void setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, unitModel.toTicks100ms(velocity / 60));
    }

    /**
     * Gets the queue of all the balls in the conveyor.
     *
     * @return the queue of the balls.
     */
    public Deque<String> getQueue() {
        return new ArrayDeque<>(cargoPositions);
    }

    public void updateCargoInQueue(DriverStation.Alliance color) {
        SmartDashboard.putBoolean("positive power", commandPower > 0);
        SmartDashboard.putBoolean("valid", !color.equals(DriverStation.Alliance.Invalid));
        if (commandPower > 0 && !color.equals(DriverStation.Alliance.Invalid)) {
            cargoPositions.removeFirstOccurrence(DriverStation.Alliance.Invalid.name());
            cargoPositions.add(color.name());
            SmartDashboard.putString("eitan", "Joe");
        } else if (commandPower < 0 && color.equals(DriverStation.Alliance.Invalid)) {
            cargoPositions.removeLastOccurrence(getFirstNotInvalid());
            cargoPositions.add(DriverStation.Alliance.Invalid.name());
        } else {
            System.out.println("Power 0");
        }
    }

    /**
     * This function updates the actual positions of the balls based on the following sensors:
     * color sensor, post flap beam, proximity sensor (in the color sensor).
     * <p>
     * Logic documentation is included in the function.
     */
    public void updateActualBallPositions() {
        DriverStation.Alliance colorIntake;

//        if (postFlapBeam.hasChanged() && !postFlapBeam.hasObject()) {
//            cargoPositions.removeFirstOccurrence(getFirstNotInvalid());
//        }

        if (colorSensor.getProximityValue() > MIN_PROXIMITY_VALUE) {
            colorIntake = colorSensor.getColor();
        } else {
            colorIntake = DriverStation.Alliance.Invalid;
        }


        if (colorSensor.hasColorChanged()) {
            updateCargoInQueue(colorIntake);
        }

//        if (preFlapBeam.hasObject() && colorSensor.getProximityValue() > MIN_PROXIMITY_VALUE) {
//            cargoPositions.add(DriverStation.Alliance.Invalid.name());
//        }
    }

    /**
     * This function returns the first value in the queue that isn't invalid.
     *
     * @return the first not invalid value.
     */
    private String getFirstNotInvalid() {
        if (cargoPositions.getLast().equals(DriverStation.Alliance.Red.name()) || cargoPositions.getLast().equals(DriverStation.Alliance.Blue.name())) {
            return cargoPositions.getLast();
        } else if (cargoPositions.getFirst().equals(DriverStation.Alliance.Red.name()) || cargoPositions.getFirst().equals(DriverStation.Alliance.Blue.name())) {
            return cargoPositions.getFirst();
        }
        return DriverStation.Alliance.Invalid.name();
    }

    /**
     * removes the string representing the cargo from the list if the cargo is ejected and adds if the cargo is consumed
     */
    @Override
    public void periodic() {
//        postFlapBeam.updateBeamBreaker();
//        preFlapBeam.updateBeamBreaker();
//        colorSensor.updateColorSensor();
//        updateActualBallPositions();

//        SmartDashboard.putString("Positions", cargoPositions.toString());
//        SmartDashboard.putNumber("Proximity", colorSensor.getProximityValue());
//        SmartDashboard.putString("Current Detected", colorSensor.getCurrentColor().name());
//        SmartDashboard.putBoolean("Pre flap", !preFlapBeam.hasObject());
//        SmartDashboard.putNumberArray("Color", colorSensor.getRawColor());
        int minutes = (int) Math.round(DriverStation.getMatchTime()) / 60;
        String minutesString = "0" + minutes;
        int seconds = (int) Math.round(DriverStation.getMatchTime()) % 60;
        String secondsString = seconds < 10 ? "0" + seconds : seconds + "";
        String timerString = minutesString + ":" + secondsString;
        SmartDashboard.putString("timer", timerString);
    }

    @Override
    public void simulationPeriodic() {

    }
}
