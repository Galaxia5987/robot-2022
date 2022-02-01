package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;

import static frc.robot.Ports.Conveyor.*;

public class Conveyor extends SubsystemBase {
    private static Conveyor INSTANCE = null;
    private final WPI_TalonFX motor = new WPI_TalonFX(Ports.Conveyor.MOTOR);
    private final Deque<String> cargoPositions = new ArrayDeque<>();
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    private final Solenoid flap = new Solenoid(PneumaticsModuleType.CTREPCM, SOLENOID);
    private final DigitalInput postFlapBeam = new DigitalInput(Ports.Conveyor.POST_FLAP_BEAM_BREAKER);
    private final DigitalInput preFlapBeam = new DigitalInput(Ports.Conveyor.PRE_FLAP_BEAM_BREAKER);
    private final ColorMatch match = new ColorMatch();
    private DriverStation.Alliance lastSeenColor = DriverStation.Alliance.Invalid;
    private boolean wasPostFlapBeamActive = true;

    private Conveyor() {
        motor.setInverted(MOTOR_INVERSION);
        motor.enableVoltageCompensation(IS_COMPENSATING_VOLTAGE);
        match.addColorMatch(Constants.Conveyor.RED);
        match.addColorMatch(Constants.Conveyor.BLUE);
        match.addColorMatch(Constants.Conveyor.NONE);
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
        ColorMatchResult result = match.matchClosestColor(color);
        Color resultColor = result.color;
        SmartDashboard.putBoolean("match", resultColor == Color.kRed);

        if (resultColor == Constants.Conveyor.RED) {
            return DriverStation.Alliance.Red;
        } else if (resultColor == Constants.Conveyor.BLUE) {
            return DriverStation.Alliance.Blue;
        } else {
            return DriverStation.Alliance.Invalid;
        }
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
        var colorIntake = getColor();
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
            cargoPositions.removeFirst();
            cargoPositions.add(DriverStation.Alliance.Invalid.name());
        }
        if (!colorIntake.equals(lastSeenColor) && !colorIntake.equals(DriverStation.Alliance.Invalid) && power > 0) {
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
        if (cargoPositions.getLast().equals(DriverStation.Alliance.Invalid.name())) {
            return cargoPositions.getFirst();
        } else if (cargoPositions.getFirst().equals(DriverStation.Alliance.Invalid.name())) {
            return cargoPositions.getLast();
        } else {
            return DriverStation.Alliance.Invalid.name();
        }
    }

    /**
     * removes the string representing the cargo from the list if the cargo is ejected and adds if the cargo is consumed
     */
    @Override
    public void periodic() {
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
        SmartDashboard.putString("position", Arrays.toString(cargoPositions.toArray(String[]::new)));
        SmartDashboard.putString("First", cargoPositions.toArray()[0].toString());
        SmartDashboard.putString("Last", cargoPositions.toArray()[1].toString());
        SmartDashboard.putBoolean("dio", preFlapBeam.get());
        System.out.println("I am working.");
    }
}
