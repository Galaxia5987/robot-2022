package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;
import frc.robot.subsystems.flap.Flap;

import java.util.ArrayDeque;
import java.util.Deque;

import static frc.robot.Constants.Conveyor.MIN_PROXIMITY_VALUE;
import static frc.robot.Ports.Conveyor.IS_COMPENSATING_VOLTAGE;
import static frc.robot.Ports.Conveyor.MOTOR_INVERSION;

public class Conveyor extends SubsystemBase {
    private static Conveyor INSTANCE = null;
    private final WPI_TalonFX motor = new WPI_TalonFX(Ports.Conveyor.MOTOR);
    private final Deque<DriverStation.Alliance> cargoPositions = new ArrayDeque<>();
    private final BeamBreaker postFlapBeam = new BeamBreaker(Ports.Conveyor.POST_FLAP_BEAM_BREAKER);
    private final BeamBreaker preFlapBeam = new BeamBreaker(Ports.Conveyor.PRE_FLAP_BEAM_BREAKER);
    private final UnitModel unitModel = new UnitModel(Constants.Conveyor.TICKS_PER_UNIT);
    private final ColorSensor colorSensor = new ColorSensor(I2C.Port.kMXP);

    private final DoubleLogEntry power;
    private final StringLogEntry positions;
    private boolean isIntaking = true;
    private double lastPower = 0.1;

    private Conveyor() {
        motor.setInverted(MOTOR_INVERSION);
        motor.enableVoltageCompensation(IS_COMPENSATING_VOLTAGE);
        motor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);

        DataLog log = DataLogManager.getLog();
        power = new DoubleLogEntry(log, "/conveyor/power");
        positions = new StringLogEntry(log, "/conveyor/positions");
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
     * Get the input of the pre-flap beam breaker.
     *
     * @return the input of the pre-flap beam breaker (true or false).
     */
    public boolean isPreFlapBeamConnected() {
        return !preFlapBeam.hasObject();
    }

    /**
     * Get input of the post-flap beam breaker.
     *
     * @return the input of the post-flap beam breaker (true or false).
     */
    public boolean isPostFlapBeamConnected() {
        return !postFlapBeam.hasObject();
    }

    /**
     * @return the amount of balls inside the cargo.
     */
    public int getCargoCount() {
        return cargoPositions.size();
    }

    /**
     * Gets the power of the motor.
     *
     * @return the power of the motor. [%]
     */
    public double getPower() {
        return motor.get();
    }

    /**
     * Set motor power.
     *
     * @param power the power to the motor [-1, 1].
     */
    public void setPower(double power) {
        motor.set(power);

        if (power != 0) {
            lastPower = power;
        }
    }

    public double getVelocity() {
        return unitModel.toVelocity(motor.getSelectedSensorVelocity()) * 60;
    }

    public void setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, unitModel.toTicks100ms(velocity / 60));
    }

    /**
     * Gets the queue of all the balls in the conveyor.
     *
     * @return the queue of the balls.
     */
    public Deque<DriverStation.Alliance> getQueue() {
        return new ArrayDeque<>(cargoPositions);
    }

    /**
     * This function updates the actual positions of the balls based on the following sensors:
     * color sensor, post flap beam, proximity sensor (in the color sensor).
     * <p>
     * Logic documentation is included in the function.
     */
    public void updateActualBallPositions() {
        boolean currentlyIntaking = getPower() >= 0;
        if (colorSensor.getProximityValue() >= MIN_PROXIMITY_VALUE && (colorSensor.hasColorChanged())) {
            var color = colorSensor.getColor();
            if ((color != DriverStation.Alliance.Invalid) == currentlyIntaking) {
                if (currentlyIntaking) {
                    cargoPositions.addLast(color);
                } else if (!cargoPositions.isEmpty()) {
                    cargoPositions.removeLast();
                }
            }
        }
        isIntaking = currentlyIntaking;

        if (postFlapBeam.hasChanged() && !postFlapBeam.hasObject() && Flap.getInstance().getFlapMode().equals(Flap.FlapMode.ALLOW_SHOOTING)) {
            if (!cargoPositions.isEmpty()) {
                cargoPositions.removeFirst();
            }
        }
    }

    public void clearStack() {
        cargoPositions.clear();
    }

    public int getColorSensorProximity() {
        return colorSensor.getProximityValue();
    }

    public DriverStation.Alliance getColor() {
        return colorSensor.getColor();
    }

    /**
     * removes the string representing the cargo from the list if the cargo is ejected and adds if the cargo is consumed
     */
    @Override
    public void periodic() {
        postFlapBeam.updateBeamBreaker();
        preFlapBeam.updateBeamBreaker();
        colorSensor.updateColorSensor();
        updateActualBallPositions();

        SmartDashboard.putString("Positions", cargoPositions.toString());
        SmartDashboard.putNumber("Proximity", colorSensor.getProximityValue());
        SmartDashboard.putBoolean("Pre flap", preFlapBeam.get());
        SmartDashboard.putBoolean("Post flap", postFlapBeam.get());
        System.out.println("Pre flap: " + preFlapBeam.get());
        System.out.println("Post flap: " + postFlapBeam.get());
        int minutes = (int) Math.round(DriverStation.getMatchTime()) / 60;
        String minutesString = "0" + minutes;
        int seconds = (int) Math.round(DriverStation.getMatchTime()) % 60;
        String secondsString = seconds < 10 ? "0" + seconds : seconds + "";
        String timerString = minutesString + ":" + secondsString;
        SmartDashboard.putString("timer", timerString);

        String firstColor = "";
        String secondColor = "";

        if (cargoPositions.size() > 0) {
            firstColor = cargoPositions.getLast().equals(DriverStation.Alliance.Blue) ? "blue" : "red";
        }

        if (cargoPositions.size() > 1) {
            secondColor = cargoPositions.getFirst().equals(DriverStation.Alliance.Blue) ? "blue" : "red";
        }

        SmartDashboard.putString("first_color", firstColor);
        SmartDashboard.putString("second_color", secondColor);

        power.append(getPower());
        positions.append(cargoPositions.toString());
    }
}
