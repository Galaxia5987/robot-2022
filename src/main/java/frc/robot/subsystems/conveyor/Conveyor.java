package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;

import static frc.robot.Constants.Conveyor.*;
import static frc.robot.Ports.Conveyor.*;

public class Conveyor extends SubsystemBase {
    private static Conveyor INSTANCE = null;
    private final WPI_TalonFX motor = new WPI_TalonFX(Ports.Conveyor.MOTOR);
    private final Deque<String> cargoPositions = new ArrayDeque<>();
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    private final Solenoid flap = new Solenoid(PneumaticsModuleType.CTREPCM, SOLENOID);
    private final DigitalInput postFlapBeam = new DigitalInput(Ports.Conveyor.POST_FLAP_BEAM_BREAKER);
    private final DigitalInput preFlapBeam = new DigitalInput(Ports.Conveyor.PRE_FLAP_BEAM_BREAKER);
    private final ColorMatch colorMatch = new ColorMatch();
    private DriverStation.Alliance lastSeenColor = DriverStation.Alliance.Invalid;
    private boolean wasPostFlapBeamConnected = true;
    private int currentProximity = 0;

    private Conveyor() {
        motor.setInverted(MOTOR_INVERSION);
        motor.enableVoltageCompensation(IS_COMPENSATING_VOLTAGE);
        colorMatch.addColorMatch(RED);
        colorMatch.addColorMatch(GREEN);
        colorMatch.addColorMatch(BLUE);
        colorMatch.addColorMatch(NONE);
        cargoPositions.add(DriverStation.Alliance.Invalid.name());
        cargoPositions.add(DriverStation.Alliance.Invalid.name());
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
     * @return the color sensor value as a {@link edu.wpi.first.wpilibj.DriverStation.Alliance} enum.
     */
    public DriverStation.Alliance getColor() {
        Color color = colorSensor.getColor();
        ColorMatchResult result = colorMatch.matchClosestColor(color);
        Color resultColor = result.color;

        if (resultColor == RED) {
            return DriverStation.Alliance.Red;
        } else if (resultColor == BLUE) {
            return DriverStation.Alliance.Blue;
        } else {
            return DriverStation.Alliance.Invalid;
        }
    }

    /**
     * Returns the proximity of the color sensor from the nearest object.
     *
     * @return the proximity from the object (0 to 2047, largest when object is close).
     */
    public int getProximityValue() {
        return colorSensor.getProximity();
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
    public boolean isPostFlapBeamConnected(){
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
     * Set motor power.
     *
     * @param power the power to the motor [-1, 1].
     */
    public void setPower(double power) {
        motor.set(power);
    }

    /**
     * open the flap
     */
    public void openFlap() {
        flap.set(false);
    }

    /**
     * closes the flap
     */
    public void closeFlap() {
        flap.set(true);
    }

    /**
     * toggles the flap
     */
    public void toggleFlap() {
        flap.toggle();
    }

    /**
     * This function updates the actual positions of the balls based on the following sensors:
     * color sensor, post flap beam, proximity sensor (in the color sensor).
     *
     * Logic documentation is included in the function.
     */
    private void updateActualBallPositions() {
        /*
        Condition: if the current proximity of the object in front of the sensor is less than the constant
            true => get the current color the color sensor sees
                    Condition: if the current input of the color sensor is invalid and the last wasn't
                        true => set the current input of the color sensor to the last input
            false => set current input of the color sensor to invalid
         */
        DriverStation.Alliance colorIntake;
        if (currentProximity >= MIN_PROXIMITY_VALUE) {
            colorIntake = getColor();
            if (colorIntake.equals(DriverStation.Alliance.Invalid) && !lastSeenColor.equals(DriverStation.Alliance.Invalid)) {
                colorIntake = lastSeenColor;
            }
        } else {
            colorIntake = DriverStation.Alliance.Invalid;
        }

        boolean isPostFlapBeamConnected = isPostFlapBeamConnected();
        double power = motor.getMotorOutputPercent();

        /*
        Condition: if the post flap beam break is active and last input it wasn't and the power is positive
            true => Condition: if the amount of non-invalid cargo in the queue is 1
                        true => remove the first non-invalid in the queue
                        false => remove the head of the queue
                    add an invalid value to the tail of the queue
         */
        if (isPostFlapBeamConnected && !wasPostFlapBeamConnected && power > 0) {
            if (getCargoCount() == 1) {
                cargoPositions.removeFirstOccurrence(getFirstNotInvalid());
            } else {
                cargoPositions.removeFirst();
            }
            cargoPositions.add(DriverStation.Alliance.Invalid.name());
        }

        /*
        Condition: if the current input of the color sensor isn't equal to the last
            true => Condition: if current input isn't invalid and the power is positive and there isn't 2 cargo in the queue
                        true => remove the first invalid value in the queue
                                add the current input of the color sensor
                        false => Condition: the current input of the color sensor is invalid and the power is negative
                                    true => remove the first non-invalid value in the queue
                                            add an invalid value at the tail of the queue
         */
        if(!colorIntake.equals(lastSeenColor)) {
            if (!colorIntake.equals(DriverStation.Alliance.Invalid) && power > 0 && getCargoCount() != MAX_CARGO_AMOUNT) {
                cargoPositions.removeFirstOccurrence(DriverStation.Alliance.Invalid.name());
                cargoPositions.add(colorIntake.name());
            } else if (colorIntake.equals(DriverStation.Alliance.Invalid) && power < 0) {
                cargoPositions.removeFirstOccurrence(getFirstNotInvalid());
                cargoPositions.add(DriverStation.Alliance.Invalid.name());
            }
        }

        wasPostFlapBeamConnected = isPostFlapBeamConnected;
        lastSeenColor = colorIntake;
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
        } else {
            return DriverStation.Alliance.Invalid.name();
        }
    }

    /**
     * removes the string representing the cargo from the list if the cargo is ejected and adds if the cargo is consumed
     */
    @Override
    public void periodic() {
        currentProximity = colorSensor.getProximity();
        updateActualBallPositions();
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putString("position", cargoPositions.toString());
        var color = colorSensor.getColor();
        var alliance = getColor();
        SmartDashboard.putNumberArray("color", new double[]{color.red, color.green, color.blue});
        System.out.println(Arrays.toString(new double[]{color.red, color.green, color.blue}));
        SmartDashboard.putString("detected-color", alliance.name());
        String[] positions = cargoPositions.toArray(String[]::new);
        SmartDashboard.putStringArray("position", positions);
        SmartDashboard.putString("First", positions[0]);
        SmartDashboard.putString("Last", positions[1]);
        SmartDashboard.putBoolean("dio", preFlapBeam.get());
    }

    public enum FlapMode {
        Open(true),
        Closed(false);

        public final boolean mode;

        FlapMode(boolean mode) {
            this.mode = mode;
        }
    }
}
