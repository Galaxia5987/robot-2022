package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

import java.util.Deque;
import java.util.LinkedList;

public class Conveyor extends SubsystemBase {
    private static Conveyor INSTANCE = null;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Conveyor.AUX);
    private DriverStation.Alliance color;
    private int cargoCount;
    private Deque<String> position = new LinkedList<>();

    private Conveyor() {
        motor.setInverted(Ports.Conveyor.IS_MAIN_INVERTED);
        setColor(DriverStation.Alliance.Red); // TODO: get from the driver station
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

    public static DriverStation.Alliance GetcolorIntake() {
        return DriverStation.Alliance.Red;
    }

    public static DriverStation.Alliance GetcolorShooter() {
        return DriverStation.Alliance.Red;
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
//                position.addFirst(colorShooter.label);
            }

        }
        if (colorIntake != DriverStation.Alliance.Invalid) {
            if (motor.getMotorOutputPercent() < 0) {
                cargoCount--;
                position.removeLast();
            } else if (motor.getMotorOutputPercent() > 0) {
                cargoCount++;
//                position.add(colorIntake.label);
            }
        }
    }

    private String enumToString(DriverStation.Alliance alliance) {
        switch (alliance) {
            case Red:
                System.out.println("RED");
            case Blue:
                System.out.println("Blue");

        }
        return "bruh";
    }

}
