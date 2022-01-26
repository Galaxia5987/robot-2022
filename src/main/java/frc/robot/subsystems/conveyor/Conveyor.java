package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

import java.util.Deque;
import java.util.LinkedList;

public class Conveyor extends SubsystemBase {
    private static Conveyor INSTANCE = null;
    private final WPI_TalonFX motor = new WPI_TalonFX(Ports.Conveyor.AUX);
    private final Deque<String> position = new LinkedList<>();
    private final ColorSensorV3 colorSensorIntake = new ColorSensorV3(I2C.Port.kOnboard);
    private final DigitalInput beamBreaker = new DigitalInput(0);
    private final Solenoid flap = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    ColorMatch match = new ColorMatch();
    private DriverStation.Alliance lastSeenColor = DriverStation.Alliance.Invalid;
    private boolean lastPassed = true;

    private Conveyor() {
        motor.setInverted(TalonFXInvertType.Clockwise);
        match.addColorMatch(Color.kRed); // red
        match.addColorMatch(Color.kBlue); // blue
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
     * @return the color sensor value as a DriverStation.Alliance enum
     */
    public DriverStation.Alliance getColor() {
        Color color = colorSensorIntake.getColor();
        System.out.println(color.red + " Red " + color.blue + " Blue " + color.green + " Green ");
        ColorMatchResult result = match.matchClosestColor(color);
        Color resultColor = result.color;

        if (resultColor == Color.kRed) {
            return DriverStation.Alliance.Red;
        } else if (resultColor == Color.kBlue) {
            return DriverStation.Alliance.Blue;
        } else {
            return DriverStation.Alliance.Invalid;
        }
    }

    /**
     * @returns the amount of balls inside the cargo
     */
    public int getCargoCount() {
        return position.size();
    }

    /**
     * Set motor power.
     * @param power the power to the motor [-1, 1].
     */
    public void setPower(double power) {
        motor.set(power);
    }

    /**
     * @param first the cargo place for which to get the color, true - first, false - last.
     * @returns the color of first/last cargos inside the conveyor.
     */
    public DriverStation.Alliance getColorPosition(boolean first) {
        if (first) {
            return DriverStation.Alliance.valueOf(position.getFirst());
        }
        return DriverStation.Alliance.valueOf(position.getLast());
    }

    /**
     * open the "flap" - solenoid
     */
    public void openFlap() {
        flap.set(false);
    }

    /**
     * closes the "flap" - solenoid
     */
    public void closeFlap() {
        flap.set(true);
    }

    /**
     * toggles the "flap" - solenoid
     */
    public void toggle() {
        flap.toggle();
    }

    /**
     *
     * @param alliance an DriverStation.Alliance enum
     * @return the name of the enum
     */
    private String enumToString(DriverStation.Alliance alliance) {
        return alliance.name();
    }

    /**
     * removes the string representing the ball from the list if the ball is ejected and adds if the ball is consumed
     */
    @Override
    public void periodic() {
        var colorIntake = getColor();
        boolean hasPassed = beamBreaker.get();
        if (hasPassed && !lastPassed && motor.getMotorOutputPercent() > 0) { //
            position.removeFirst();
        }
        if (colorIntake == DriverStation.Alliance.Invalid && lastSeenColor != DriverStation.Alliance.Invalid) {
            if (motor.getMotorOutputPercent() < 0) {
                position.removeLast();
            } else if (motor.getMotorOutputPercent() > 0) {
                position.add(enumToString(colorIntake));
            }
        }
        lastPassed = hasPassed;
        lastSeenColor = colorIntake;
    }

}