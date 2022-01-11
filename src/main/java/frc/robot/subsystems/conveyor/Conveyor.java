package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Ports;

public class Conveyor {
    private static Conveyor INSTANCE = null;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(Ports.Conveyor.AUX);
    private AllianceColor color;


    private Conveyor() {
        motor.setInverted(Ports.Conveyor.IS_MAIN_INVERTED);
        setColor(AllianceColor.RED);
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
    public void setpower(double power){
        motor.set(power);
    }

    public static AllianceColor Getcolor(){
    }

    public boolean Iscargoalliencecolor(){
        return color == Getcolor();
    }

    public void setColor(AllianceColor fmsColor) {
        color = fmsColor;
    }
    public void Counter(int counter){
        if (Getcolor() == AllianceColor.RED || Getcolor() == AllianceColor.BLUE){
            counter++;
            return counter
        }
    }

    public enum AllianceColor {
        RED,
        BLUE,
        OTHER
    }

}
