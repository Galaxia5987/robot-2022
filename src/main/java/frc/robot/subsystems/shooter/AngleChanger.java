package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Ports.Shooter.SOLENOID;

public class AngleChanger extends SubsystemBase {
    private static final AngleChanger INSTANCE = new AngleChanger();
    private final Solenoid angleChanger = new Solenoid(PneumaticsModuleType.CTREPCM, SOLENOID);

    public AngleChanger() {
    }

    public static AngleChanger getINSTANCE() {
        return INSTANCE;
    }

    public void changeAngle() {
        angleChanger.set(!angleChanger.get());
    }

    public boolean getActive() {
        return angleChanger.get();
    }
}
