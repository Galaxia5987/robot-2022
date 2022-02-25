package frc.robot.autoPaths;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commandgroups.ShootCargo;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class TaxiFromUpUpAndShoot extends SequentialCommandGroup {
    private DoubleSupplier distanceFromTarget;
    private DoubleSupplier conveyorPower;

    // Taxi from up up tarmac, shoot pre-loaded cargo, park near up tarmac.(2)
    public TaxiFromUpUpAndShoot(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap) {
        var rotationPID = new ProfiledPIDController(Constants.Autonomous.KP_THETA_CONTROLLER, 0, 0, new TrapezoidProfile.Constraints(Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL));
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        Function<String, PPSwerveControllerCommand> createCommand = path -> new PPSwerveControllerCommand(
                PathPlanner.loadPath(path, Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL),
                swerveDrive::getPose,
                swerveDrive.getKinematics(),
                new PIDController(Constants.Autonomous.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Autonomous.KP_Y_CONTROLLER, 0, 0),
                rotationPID,
                swerveDrive::setStates,
                swerveDrive);


        addCommands(createCommand.apply("p0 - Taxi from up up tarmac(2.1"));

        addCommands(new ShootCargo(
                shooter,
                hood,
                conveyor,
                flap,
                conveyorPower,
                distanceFromTarget)
                .withTimeout(3));

        addCommands(createCommand.apply("p0 - Go to up tarmac(1.2.2)"));
    }
}
