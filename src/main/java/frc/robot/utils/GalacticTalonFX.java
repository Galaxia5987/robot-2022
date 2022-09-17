package frc.robot.utils;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Class for custom functionality for the Talon FX, Falcon 500's controller.
 */
public class GalacticTalonFX extends TalonFX {

    private double unitsCoefficient;

    /**
     * Instantiates a Talon FX.
     * <p>
     * Reports configuration errors to the Driver Station.
     *
     * @param deviceNumber  device number (ID) to assign to the Talon FX
     * @param configuration configuration for the Talon FX
     */
    public GalacticTalonFX(int deviceNumber, TalonFXConfiguration configuration) {
        this(deviceNumber, configuration, 1, "");
    }

    /**
     * Instantiates a Talon FX.
     * <p>
     * Reports configuration errors to the Driver Station.
     *
     * @param deviceNumber     device number (ID) to assign to the Talon FX
     * @param configuration    configuration for the Talon FX
     * @param unitsCoefficient units ratio coefficient, e.g, to convert internal velocity units to m/s
     */
    public GalacticTalonFX(int deviceNumber, TalonFXConfiguration configuration, double unitsCoefficient) {
        this(deviceNumber, configuration, unitsCoefficient, "");
    }

    /**
     * Instantiates a Talon FX.
     * <p>
     * Reports configuration errors to the Driver Station.
     *
     * @param deviceNumber     device number (ID) to assign to the Talon FX
     * @param configuration    configuration for the Talon FX
     * @param canBus           Name of the CANbus; can be a SocketCAN interface (on Linux),
     *                         or a CANivore device name or serial number
     * @param unitsCoefficient units ratio coefficient, e.g, to convert internal velocity units to m/s
     */
    public GalacticTalonFX(int deviceNumber, TalonFXConfiguration configuration, double unitsCoefficient, String canBus) {
        super(deviceNumber, canBus);
        this.unitsCoefficient = unitsCoefficient;

        ErrorCode errorCode = configAllSettings(configuration);
        if (errorCode != ErrorCode.OK) {
            String errorMessage = String.format("Error occurred while creating TalonFX with ID %d: ", deviceNumber) + errorCode.toString();
            DriverStation.reportError(errorMessage, true);
        }
    }

    /**
     * Gets the selected sensor position (in converted units).
     *
     * @return Position of selected sensor (in converted units).
     */
    @Override
    public double getSelectedSensorPosition() {
        return unitsCoefficient * super.getSelectedSensorPosition();
    }

    /**
     * Gets the selected sensor position (in converted units).
     *
     * @param pidIdx 0 for Primary closed-loop. 1 for auxiliary closed-loop.
     *               See Phoenix-Documentation for how to interpret.
     * @return Position of selected sensor (in converted units).
     */
    @Override
    public double getSelectedSensorPosition(int pidIdx) {
        return unitsCoefficient * super.getSelectedSensorPosition(pidIdx);
    }
}
