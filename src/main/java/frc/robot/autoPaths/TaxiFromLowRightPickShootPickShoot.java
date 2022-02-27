package frc.robot.autoPaths;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import frc.robot.subsystems.intake.commands.IntakeByRobotSpeed;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.PhotonVisionModule;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class TaxiFromLowRightPickShootPickShoot extends SequentialCommandGroup {

    // Taxi from low right tarmac, pickup low cargo, shoot, pick up middle cargo, shoot, park near low tarmac.(7)
    public TaxiFromLowRightPickShootPickShoot(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        DoubleSupplier distanceFromTarget = () -> visionModule.getDistance().orElse(-Constants.Vision.TARGET_RADIUS) + Constants.Vision.TARGET_RADIUS;
        DoubleSupplier conveyorPower = () -> Constants.Conveyor.SHOOT_POWER;

        Function<String, FollowPath> createCommand = path -> new FollowPath(
                PathPlanner.loadPath(path, Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL),
                swerveDrive::getPose,
                swerveDrive.getKinematics(),
                new PIDController(Constants.Autonomous.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Autonomous.KP_Y_CONTROLLER, 0, 0),
                swerveDrive::setStates,
                swerveDrive);

        addCommands(new ParallelRaceGroup(
                createCommand.apply("p2 - Taxi from low right tarmac and pickup low cargo(7.1)"),
                new PickUpCargo(
                        conveyor,
                        flap,
                        intake,
                        Constants.Conveyor.DEFAULT_POWER.get(),
                        Constants.Intake.DEFAULT_POWER::get
                )));

        addCommands(
                new Convey(conveyor, -conveyorPower.getAsDouble()).withTimeout(0.05),
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

        addCommands(
                new ParallelRaceGroup(
                        createCommand.apply("p2 - Picking up middle cargo(7.2)"),
                        new PickUpCargo(
                                conveyor,
                                flap,
                                intake,
                                Constants.Conveyor.DEFAULT_POWER.get(),
                                Constants.Intake.DEFAULT_POWER::get
                        ),
                        new RunCommand(() -> shooter.setVelocity(Shoot.getSetpointVelocity(4, false)))
                )
        );

        addCommands(
                new Convey(conveyor, -conveyorPower.getAsDouble()).withTimeout(0.05),
                new ParallelRaceGroup(
                        new IntakeByRobotSpeed(intake, () -> 0),
                        new ShootCargo(
                                shooter,
                                hood,
                                conveyor,
                                flap,
                                conveyorPower,
                                distanceFromTarget)
                                .withTimeout(3),
                        new SimpleAdjustWithVision(swerveDrive, () -> 0, () -> true, () -> visionModule.getYaw().orElse(0), distanceFromTarget)
                )
        );
    }
}
