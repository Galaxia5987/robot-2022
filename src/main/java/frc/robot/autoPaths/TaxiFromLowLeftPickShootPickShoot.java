package frc.robot.autoPaths;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commandgroups.PickUpCargo;
import frc.robot.commandgroups.ShootCargo;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.auto.FollowPath;
import frc.robot.subsystems.drivetrain.commands.testing.SimpleAdjustWithVision;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class TaxiFromLowLeftPickShootPickShoot extends SequentialCommandGroup {


    // Taxi from low left tarmac, pickup middle cargo, shoot, pick up low cargo, shoot, park in low tarmac.(6)
    public TaxiFromLowLeftPickShootPickShoot(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        DoubleSupplier distanceFromTarget = visionModule::getDistance;
        DoubleSupplier conveyorPower = () -> Constants.Conveyor.SHOOT_POWER;
        var rotationPID = new ProfiledPIDController(Constants.Autonomous.KP_THETA_CONTROLLER, 0, 0, new TrapezoidProfile.Constraints(Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL));
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        Function<String, FollowPath> createCommand = path -> new FollowPath(
                PathPlanner.loadPath(path, Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL),
                swerveDrive::getPose,
                swerveDrive.getKinematics(),
                new PIDController(Constants.Autonomous.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Autonomous.KP_Y_CONTROLLER, 0, 0),
                swerveDrive::setStates,
                swerveDrive);

        addCommands(new ParallelRaceGroup((createCommand.apply("p2 - Taxi from low left tarmac and pickup middle cargo(6.1)")),
                new PickUpCargo(
                        conveyor,
                        flap,
                        intake,
                        Constants.Conveyor.DEFAULT_POWER.get(),
                        Constants.Intake.DEFAULT_POWER::get
                )));

        addCommands(
                new Convey(conveyor, -conveyorPower.getAsDouble()).withTimeout(0.1),
                new ParallelRaceGroup(
                        new ShootCargo(
                                shooter,
                                hood,
                                conveyor,
                                flap,
                                conveyorPower,
                                distanceFromTarget)
                                .withTimeout(5),
                        new SimpleAdjustWithVision(swerveDrive, () -> 0, () -> true, () -> visionModule.getYaw().orElse(0), distanceFromTarget))
        );

        addCommands(new ParallelRaceGroup((createCommand.apply("p2 - Picking up low cargo(6.2)")),
                new PickUpCargo(
                        conveyor,
                        flap,
                        intake,
                        Constants.Conveyor.DEFAULT_POWER.get(),
                        Constants.Intake.DEFAULT_POWER::get
                ).withTimeout(3)));

        addCommands(new Convey(conveyor, -conveyorPower.getAsDouble()).withTimeout(0.1),
                new ParallelRaceGroup(
                        new ShootCargo(
                                shooter,
                                hood,
                                conveyor,
                                flap,
                                conveyorPower,
                                distanceFromTarget)
                                .withTimeout(3),
                        new SimpleAdjustWithVision(swerveDrive, () -> 0, () -> true, () -> visionModule.getYaw().orElse(0), distanceFromTarget)));
    }
}
