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
    private boolean wasPostFlapBeamActive = true;
    private int currentDistance = 0;

    private Conveyor() {
        motor.setInverted(MOTOR_INVERSION);
        motor.enableVoltageCompensation(IS_COMPENSATING_VOLTAGE);
        colorMatch.addColorMatch(RED);
        colorMatch.addColorMatch(BLUE);
        colorMatch.addColorMatch(GREEN);
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
        SmartDashboard.putBoolean("match", resultColor == Color.kRed);

        if (resultColor == RED) {
            return DriverStation.Alliance.Red;
        } else if (resultColor == BLUE) {
            return DriverStation.Alliance.Blue;
        } else {
            return DriverStation.Alliance.Invalid;
        }
    }

    public int getProximityValue() {
        return colorSensor.getProximity();
    }

    public boolean getPreFlapBeam() {
        return preFlapBeam.get();
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

    private void updateActualBallPositions() {
        DriverStation.Alliance colorIntake;
        SmartDashboard.putNumber("distance", currentDistance);
        if (currentDistance >= MIN_PROXIMITY_VALUE) {
            colorIntake = getColor();
            SmartDashboard.putBoolean("MIN_VALUE", true);
            if (colorIntake.equals(DriverStation.Alliance.Invalid) && !lastSeenColor.equals(DriverStation.Alliance.Invalid)) {
                colorIntake = lastSeenColor;
            } else if (colorIntake.equals(DriverStation.Alliance.Invalid)) {
                cargoPositions.removeFirstOccurrence(DriverStation.Alliance.Invalid.name()); // remove invalid/unknown
                cargoPositions.add("Unknown");
            }
        } else {
            colorIntake = DriverStation.Alliance.Invalid;
            SmartDashboard.putBoolean("MIN_VALUE", false);
        }

        boolean isPostFlapBeamActive = postFlapBeam.get();
        SmartDashboard.putString("alliance", colorIntake.name());
        double power = motor.getMotorOutputPercent();
        /*
        Condition: post flap beam is currently active and wasn't in the last input and velocity is positive
            true => Remove the head of the queue
               Add an invalid value at the tail of the queue
               
        Condition: cargo coming into the conveyor
            true => Remove the first invalid value
               Add the new color sensor input
            false => Condition: ball is coming out of the conveyor
                    true => Remove the first non-invalid value from the queue
                            Add an invalid value into the queue
         */
        if (isPostFlapBeamActive && !wasPostFlapBeamActive && power > 0) {
            if (getCargoCount() == 1) {
                cargoPositions.removeFirstOccurrence(getFirstNonInvalid());
            } else {
                cargoPositions.removeFirst();
            }
            cargoPositions.add(DriverStation.Alliance.Invalid.name());
        }
        if (!colorIntake.equals(lastSeenColor) && !colorIntake.equals(DriverStation.Alliance.Invalid) && power > 0 && getCargoCount() != 2) {
            cargoPositions.removeFirstOccurrence(DriverStation.Alliance.Invalid.name());
            cargoPositions.add(colorIntake.name());
        } else if (!colorIntake.equals(lastSeenColor) && colorIntake.equals(DriverStation.Alliance.Invalid) && power < 0) {
            cargoPositions.removeFirstOccurrence(getFirstNonInvalid());
            cargoPositions.add(DriverStation.Alliance.Invalid.name());
        }
        wasPostFlapBeamActive = isPostFlapBeamActive;
        lastSeenColor = colorIntake;
        SmartDashboard.putString("lastSeen", lastSeenColor.name());
    }

    /**
     * This function returns the first value in the queue that isn't invalid.
     *
     * @return the first non-invalid value.
     */
    private String getFirstNonInvalid() {
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
        currentDistance = colorSensor.getProximity();
        updateActualBallPositions();
        var color = colorSensor.getColor();
        SmartDashboard.putNumberArray("color", new double[]{color.blue, color.red, color.green});
        SmartDashboard.putString("color-name", getColor().name());
        SmartDashboard.putString("positions", Arrays.toString(cargoPositions.toArray()));
        SmartDashboard.putString("ima-shel-alon", colorSensor.getRed() + "      " + colorSensor.getGreen() + "      " + colorSensor.getBlue());
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putString("position", cargoPositions.toString());
        var color = colorSensor.getColor();
        var alliance = getColor();
        SmartDashboard.putNumberArray("color", new double[]{color.red, color.green, color.blue});
        System.out.println(Arrays.toString(new double[]{color.red, color.green, color.blue}));
        SmartDashboard.putString("detected-color", alliance.name());
        SmartDashboard.putString("position", Arrays.toString(cargoPositions.toArray(String[]::new)));
        SmartDashboard.putString("First", cargoPositions.toArray()[0].toString());
        SmartDashboard.putString("Last", cargoPositions.toArray()[1].toString());
        SmartDashboard.putBoolean("dio", preFlapBeam.get());
        System.out.println("I am working.");
    }
}
