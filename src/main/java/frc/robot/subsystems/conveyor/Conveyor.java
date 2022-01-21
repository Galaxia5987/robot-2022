package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

import java.util.Deque;
import java.util.LinkedList;
import com.revrobotics.ColorSensorV3;

public class Conveyor extends SubsystemBase {
    private static Conveyor INSTANCE = null;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Conveyor.AUX);
    private DriverStation.Alliance color;
    private int cargoCount;
    private final Deque<String> position = new LinkedList<>();
    private final ColorSensorV3 colorSensorIntake = new ColorSensorV3(I2C.Port.kMXP);
    private final ColorSensorV3 colorSensorShooter = new ColorSensorV3(I2C.Port.kOnboard);
    ColorMatch match = new ColorMatch();

    private Conveyor() {
        motor.setInverted(Ports.Conveyor.IS_MAIN_INVERTED);
        setColor(DriverStation.Alliance.Red); // TODO: get from the driver station
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

    public DriverStation.Alliance GetcolorIntake() {
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

    public DriverStation.Alliance GetcolorShooter() {
        Color color = colorSensorShooter.getColor();
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

    public int getCargoCount() {
        return cargoCount;
    }

    public void setPower(double power) {
        motor.set(power);
    }

//    public boolean Iscargoalliencecolor() {
//        return color == Getcolor();
//    }

    public void setColor(DriverStation.Alliance fmsColor) {
        color = fmsColor;
    }

    public DriverStation.Alliance getColorPosition(boolean first) {
        if (first) {
            return DriverStation.Alliance.valueOf(position.getFirst());
        }
        return DriverStation.Alliance.valueOf(position.getLast());
    }

    @Override
    public void periodic() {
        var colorShooter = GetcolorShooter();
        var colorIntake = GetcolorIntake();
        if (colorShooter != DriverStation.Alliance.Invalid) {
            if (motor.getMotorOutputPercent() > 0) {
                cargoCount--;
                position.removeFirst();
            } else if (motor.getMotorOutputPercent() < 0) {
                cargoCount++;
                position.addFirst(enumToString(colorShooter));
            }

        }
        if (colorIntake != DriverStation.Alliance.Invalid) {
            if (motor.getMotorOutputPercent() < 0) {
                cargoCount--;
                position.removeLast();
            } else if (motor.getMotorOutputPercent() > 0) {
                cargoCount++;
                position.add(enumToString(colorIntake));
            }
        }
    }

    private String enumToString(DriverStation.Alliance alliance) {
        return alliance.name();
    }

}
