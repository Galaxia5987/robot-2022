package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

import java.util.Arrays;
import java.util.concurrent.ArrayBlockingQueue;

import static frc.robot.Constants.Conveyor.MAX_CARGO_AMOUNT;
import static frc.robot.Ports.Conveyor.MOTOR_INVERSION;

public class Conveyor extends SubsystemBase {
    private static Conveyor INSTANCE = null;
    private final WPI_TalonFX motor = new WPI_TalonFX(Ports.Conveyor.MOTOR);
    private final ArrayBlockingQueue<String> cargoPositions = new ArrayBlockingQueue<>(MAX_CARGO_AMOUNT);
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    private final DigitalInput postFlapBeam = new DigitalInput(Ports.Conveyor.POST_FLAP_BEAM_BREAKER);
    private final DigitalInput preFlapBeam = new DigitalInput(Ports.Conveyor.PRE_FLAP_BEAM_BREAKER);
    private final ColorMatch match = new ColorMatch();
    private DriverStation.Alliance lastSeenColor = DriverStation.Alliance.Invalid;
    private boolean wasPostFlapBeamActive = true;

    private Conveyor() {
        motor.setInverted(MOTOR_INVERSION);
        match.addColorMatch(Constants.Conveyor.RED);
        match.addColorMatch(Constants.Conveyor.BLUE);
        match.addColorMatch(Constants.Conveyor.NONE);
        cargoPositions.add(DriverStation.Alliance.Invalid.name());
        cargoPositions.add(DriverStation.Alliance.Invalid.name());
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
     * @return the color sensor value as a DriverStation.Alliance enum.
     */
    public DriverStation.Alliance getColor() {
        Color color = colorSensor.getColor();
        ColorMatchResult result = match.matchClosestColor(color);
        Color resultColor = result.color;
        SmartDashboard.putBoolean("match", resultColor == Color.kRed);

        if (resultColor == Constants.Conveyor.RED) {
            return DriverStation.Alliance.Red;
        } else if (resultColor == Constants.Conveyor.BLUE) {
            return DriverStation.Alliance.Blue;
        } else {
            return DriverStation.Alliance.Invalid;
        }
    }

    /**
     * @return the amount of balls inside the cargo.
     */
    public int getCargoCount() {
        return cargoPositions.size();
    }

    /**
     * Set motor power.
     *
     * @param power the power to the motor [-1, 1].
     */
    public void setPower(double power) {
        motor.set(power);
    }

    /**
     * open the "flap" - solenoid
     */
    public void openFlap() {
//        flap.set(false);
    }

    /**
     * closes the "flap" - solenoid
     */
    public void closeFlap() {
//        flap.set(true);
    }

    /**
     * toggles the "flap" - solenoid
     */
    public void toggle() {
//        flap.toggle();
    }

    private void updateActualBallPositions() throws InterruptedException {
        var colorIntake = getColor();
        boolean isPostFlapBeamActive = postFlapBeam.get();
        SmartDashboard.putString("alliance", colorIntake.name());
        double power = 1;
//        double power = motor.getMotorOutputPercent();
        if (isPostFlapBeamActive && !wasPostFlapBeamActive && power > 0) {
            if(!cargoPositions.toArray()[0].equals(DriverStation.Alliance.Invalid.name())) {
                cargoPositions.poll();
                var temp = cargoPositions.take();
                cargoPositions.add(DriverStation.Alliance.Invalid.name());
                cargoPositions.add(temp);
            } else {
                cargoPositions.poll();
                cargoPositions.poll();
                cargoPositions.add(DriverStation.Alliance.Invalid.name());
                cargoPositions.add(DriverStation.Alliance.Invalid.name());
            }
        }
        if (colorIntake != lastSeenColor && !colorIntake.equals(DriverStation.Alliance.Invalid)) {
            if (power < 0) {
                if (cargoPositions.isEmpty()) {
                    cargoPositions.poll();
                }
                cargoPositions.offer(colorIntake.name());
            } else if (power > 0) {
                cargoPositions.remove();
                cargoPositions.offer(colorIntake.name());
            }
        }
        wasPostFlapBeamActive = isPostFlapBeamActive;
        lastSeenColor = colorIntake;
        SmartDashboard.putString("lastSeen", lastSeenColor.name());
    }

    private Queue getQueue() {
        /*
        In each case the first element is the last in,
        and the second is the first in.
        Case #1  queue = [None, None]
        Case #2  queue = [None, Alliance]
        Case #3  queue = [Alliance, None]
        Case #4  queue = [Alliance, Alliance]
        Case #5  queue = [Alliance, Opponent]
        Case #6  queue = [None, Opponent]
        Case #7  queue = [Opponent, None]
        Case #8  queue = [Opponent, Opponent]
        Case #9  queue = [Opponent, Alliance]
         */

        var last = cargoPositions.toArray()[1];
        var first = cargoPositions.toArray()[0];
        var invalid = DriverStation.Alliance.Invalid.name();
        var alliance = DriverStation.getAlliance().name();

        if (last.equals(invalid)) {
            if (first.equals(invalid)) {
                return Queue.NoneNone;
            } else if (first.equals(alliance)) {
                return Queue.NoneAlliance;
            }
            return Queue.NoneOpponent;
        } else if (last.equals(alliance)) {
            if (first.equals(invalid)) {
                return Queue.AllianceNone;
            } else if (first.equals(alliance)) {
                return Queue.AllianceAlliance;
            }
            return Queue.AllianceOpponent;
        }
        if (first.equals(invalid)) {
            return Queue.OpponentNone;
        } else if (first.equals(alliance)) {
            return Queue.OpponentAlliance;
        }
        return Queue.OpponentOpponent;
    }

    /**
     * removes the string representing the cargo from the list if the cargo is ejected and adds if the cargo is consumed
     */
    @Override
    public void periodic() {
        try {
            updateActualBallPositions();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putString("position", cargoPositions.toString());
        Queue queue = getQueue();
        var color = colorSensor.getColor();
        var alliance = getColor();
        SmartDashboard.putNumberArray("color", new double[]{color.red, color.green, color.blue});
        System.out.println(Arrays.toString(new double[]{color.red, color.green, color.blue}));
        SmartDashboard.putString("detected-color", alliance.name());
        SmartDashboard.putString("position", Arrays.toString(cargoPositions.toArray(String[]::new)));
        SmartDashboard.putString("Queue", Arrays.toString(queue.queue));
        SmartDashboard.putString("First", cargoPositions.toArray()[0].toString());
        SmartDashboard.putString("Last", cargoPositions.toArray()[1].toString());
        SmartDashboard.putBoolean("dio", preFlapBeam.get());
        System.out.println("I am working.");
    }

    public enum Queue {
        /*
        None - 0
        Alliance - 1
        Opponent - 2
         */
        NoneNone(new int[]{0, 0}),
        NoneAlliance(new int[]{0, 1}),
        AllianceNone(new int[]{1, 0}),
        AllianceAlliance(new int[]{1, 1}),
        AllianceOpponent(new int[]{1, 2}),
        NoneOpponent(new int[]{0, 2}),
        OpponentNone(new int[]{2, 0}),
        OpponentOpponent(new int[]{2, 2}),
        OpponentAlliance(new int[]{2, 1});

        public final int[] queue;

        Queue(int[] queue) {
            this.queue = queue;
        }
    }
}
