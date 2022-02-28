package frc.robot.autoPaths;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

public class TaxiFromLowRightPickShoot extends SequentialCommandGroup {

    // Taxi from low right, pick up low cargo, shoot, go near low tarmac.(4)
    public TaxiFromLowRightPickShoot(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule module) {
        DoubleSupplier distanceFromTarget = module::getDistance;
        DoubleSupplier conveyorPower = Constants.Conveyor.DEFAULT_POWER::get;

        Function<String, FollowPath> createCommand = path -> new FollowPath(
                PathPlanner.loadPath(path, Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL),
                swerveDrive::getPose,
                swerveDrive.getKinematics(),
                new PIDController(Constants.Autonomous.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Autonomous.KP_Y_CONTROLLER, 0, 0),
                swerveDrive::setStates,
                swerveDrive);
        addCommands(new InstantCommand(() -> module.setLeds(false)));

        addCommands(new ParallelCommandGroup(
                createCommand.apply("p1 - Taxi from low right and pickup low cargo(4.1)"),
                new PickUpCargo(
                        conveyor,
                        flap,
                        intake,
                        Constants.Conveyor.DEFAULT_POWER.get(),
                        Constants.Intake.DEFAULT_POWER::get
                ).withTimeout(3)));

        addCommands(new InstantCommand(swerveDrive::terminate));
        addCommands(
                new Convey(conveyor, -conveyorPower.getAsDouble()).withTimeout(0.05),
                new ParallelCommandGroup(
                        new ShootCargo(
                                shooter,
                                hood,
                                conveyor,
                                flap,
                                conveyorPower,
                                distanceFromTarget)
                                .withTimeout(3),
                        new SimpleAdjustWithVision(swerveDrive, () -> 0, () -> true, () -> module.getYaw().orElse(0), distanceFromTarget))
        );


        addCommands(createCommand.apply("p1 - Going to low tarmac(4.2.1)"));
    }
}

