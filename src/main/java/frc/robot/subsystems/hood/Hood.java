package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Ports.Hood.SOLENOID;

public class Hood extends SubsystemBase {
    private static Hood INSTANCE;
    private final Solenoid angleChanger = new Solenoid(PneumaticsModuleType.CTREPCM, SOLENOID);

    public Hood() {
    }

    public static Hood getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Hood();
        }
        return INSTANCE;
    }

    public void open() {
        if(angleChanger.get())
            angleChanger.toggle();

    }

    public void close() {
        if(!angleChanger.get()){
            angleChanger.toggle();
        }
    }

    public boolean getActive() {
        return angleChanger.get();
    }
}
