package frc.robot.autoPaths;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.subsystems.intake.commands.IntakeByRobotSpeed;
import frc.robot.subsystems.intake.commands.IntakeCargo;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.IntFunction;

public class SaarIsAutonomous extends SequentialCommandGroup {
    protected final SwerveDrive swerveDrive;
    protected final Shooter shooter;
    protected final Conveyor conveyor;
    protected final Intake intake;
    protected final Hood hood;
    protected final Flap flap;
    protected final PhotonVisionModule visionModule;
    protected final Function<String, FollowPath> followPath;
    protected final IntFunction<CommandBase> shootAndAdjust;
    protected final IntFunction<CommandBase> pickup;

    public SaarIsAutonomous(SwerveDrive swerveDrive, Shooter shooter, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        this.swerveDrive = swerveDrive;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.intake = intake;
        this.hood = hood;
        this.flap = flap;
        this.visionModule = visionModule;

        DoubleSupplier distanceFromTarget = visionModule::getDistance;
        DoubleSupplier conveyorPower = Constants.Conveyor.DEFAULT_POWER::get;

        followPath = path -> new FollowPath(
                PathPlanner.loadPath(path, Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL),
                swerveDrive::getPose,
                swerveDrive.getKinematics(),
                new PIDController(Constants.Autonomous.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Autonomous.KP_Y_CONTROLLER, 0, 0),
                swerveDrive::setStates,
                swerveDrive);

        shootAndAdjust = timeout -> new SequentialCommandGroup(
                new Convey(conveyor, -conveyorPower.getAsDouble()).withTimeout(0.1),
                new SimpleAdjustWithVision(swerveDrive, () -> 0, () -> true, () -> visionModule.getYaw().orElse(0), distanceFromTarget).withTimeout(0.7),
                new ParallelRaceGroup(new ShootCargo(
                        shooter,
                        hood,
                        conveyor,
                        flap,
                        conveyorPower,
                        distanceFromTarget)
                        .withTimeout(timeout),
                        new IntakeCargo(intake, Constants.Intake.DEFAULT_POWER::get),
                        new SimpleAdjustWithVision(swerveDrive, () -> 0, () -> true, () -> visionModule.getYaw().orElse(0), distanceFromTarget))
        );

        pickup = timeout -> new PickUpCargo(
                conveyor,
                flap,
                intake,
                Constants.Conveyor.DEFAULT_POWER.get(),
                Constants.Intake.DEFAULT_POWER::get
        ).withTimeout(timeout);

        addCommands(new InstantCommand(() -> visionModule.setLeds(false)));
    }
}
