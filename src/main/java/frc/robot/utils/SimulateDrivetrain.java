package frc.robot.utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SimulateDrivetrain extends SubsystemBase {
    private final EncoderSim leftEncoderSim;
    private final EncoderSim rightEncoderSim;

    private final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
            DCMotor.getFalcon500(2),
            1,
            7.5,
            60,
            0.04,
            0.59,
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    private final Field2d field = new Field2d();
    private final Field2d target = new Field2d();

    public SimulateDrivetrain() {
        Encoder leftEncoder = new Encoder(0, 1);
        Encoder rightEncoder = new Encoder(2, 3);

        leftEncoder.setDistancePerPulse(2 * Math.PI * 0.04);
        rightEncoder.setDistancePerPulse(2 * Math.PI * 0.04);

        leftEncoderSim = new EncoderSim(leftEncoder);
        rightEncoderSim = new EncoderSim(rightEncoder);
    }

    /**
     * Sets the values of movement for the simulated drivetrain.
     *
     * @param forward  is the forward output. [%]
     * @param rotation is the rotation output. [%]
     */
    public void set(double forward, double rotation) {
        rotation *= 0.3;
        forward *= 0.7;

        double outputR = forward - rotation;
        double outputL = forward + rotation;

        leftEncoderSim.setRate(outputL * Math.PI);
        rightEncoderSim.setRate(outputR * Math.PI);
    }

    /**
     * Gets the pose of the simulated drivetrain.
     *
     * @return the pose of the simulated drivetrain.
     */
    public Pose2d getPose() {
        return driveSim.getPose();
    }

    @Override
    public void simulationPeriodic() {
        driveSim.setInputs(leftEncoderSim.getRate() / Math.PI * 12,
                rightEncoderSim.getRate() / Math.PI * 12);
        driveSim.update(0.02);
        SmartDashboard.putData("Field", field);
        field.setRobotPose(driveSim.getPose());
        SmartDashboard.putData("Target", target);
//        target.setRobotPose(HUB_POSE);
    }
}
