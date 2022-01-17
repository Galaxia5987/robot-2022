package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

import java.util.Deque;
import java.util.LinkedList;

public class Conveyor extends SubsystemBase {
    private static Conveyor INSTANCE = null;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Conveyor.AUX);
    private AllianceColor color;
    private int cargoCount;
    private Deque<String> position = new LinkedList<>();

    private Conveyor() {
        motor.setInverted(Ports.Conveyor.IS_MAIN_INVERTED);
        setColor(AllianceColor.RED); // TODO: get from the driver station
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

    public static AllianceColor GetcolorIntake() {
        return AllianceColor.RED;
    }

    public static AllianceColor GetcolorShooter() {
        return AllianceColor.RED;
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

    public void setColor(AllianceColor fmsColor) {
        color = fmsColor;
    }


    @Override
    public void periodic() {
        var colorShooter = GetcolorShooter();
        var colorIntake = GetcolorIntake();
        if (colorShooter != AllianceColor.OTHER) {
            if (motor.getMotorOutputPercent() > 0) {
                cargoCount--;
                position.removeFirst();
            } else if (motor.getMotorOutputPercent() < 0) {
                cargoCount++;
                position.addFirst(colorShooter.label);
            }

        }
        if (colorIntake != AllianceColor.OTHER) {
            if (motor.getMotorOutputPercent() < 0) {
                cargoCount--;
                position.removeLast();
            } else if (motor.getMotorOutputPercent() > 0) {
                cargoCount++;
                position.add(colorIntake.label);
            }
        }
    }

    public enum AllianceColor {
        RED("red"),
        BLUE("blue"),
        OTHER("other");


        public final String label;

        private AllianceColor(String label) {
            this.label = label;
        }
    }
}
