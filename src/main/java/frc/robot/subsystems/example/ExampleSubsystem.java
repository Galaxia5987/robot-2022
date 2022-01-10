package frc.robot.subsystems.example;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class ExampleSubsystem extends SubsystemBase {
    private static ExampleSubsystem INSTANCE = null;
    // create fields here
    private final TalonFX main = new TalonFX(Ports.ExampleSubsystem.MAIN);
    private final TalonSRX aux = new TalonSRX(Ports.ExampleSubsystem.AUX);

    private ExampleSubsystem() {
        // motor and sensor inversions
        main.setInverted(Ports.ExampleSubsystem.IS_MAIN_INVERTED);
        aux.setInverted(Ports.ExampleSubsystem.IS_AUX_INVERTED);
        main.setSensorPhase(Ports.ExampleSubsystem.IS_MAIN_SENSOR_INVERTED);
        aux.setSensorPhase(Ports.ExampleSubsystem.IS_AUX_SENSOR_INVERTED);

        // follow
        aux.follow(main);
    }

    /**
     * lazy instantiation
     *
     * @return the subsystem instance
     */
    public static ExampleSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ExampleSubsystem();
        }
        return INSTANCE;
    }

    /**
     * Set the power output of the motor.
     *
     * @param power the output of the motor in percent [-1, 1].
     */
    public void setPower(double power) {
        main.set(ControlMode.PercentOutput, power);
    }

}
