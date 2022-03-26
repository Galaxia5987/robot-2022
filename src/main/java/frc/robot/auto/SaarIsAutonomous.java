package frc.robot.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commandgroups.PickUpCargo;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.ConveyCargo;
import frc.robot.subsystems.conveyor.commands.OldConveyNoWait;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.AdjustToTargetOnCommand;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.drivetrain.commands.auto.FollowPath;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeCargo;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.BackAndShootCargo;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.PhotonVisionModule;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SaarIsAutonomous extends SequentialCommandGroup {
    protected final SwerveDrive swerveDrive;
    protected final Shooter shooter;
    protected final Conveyor conveyor;
    protected final Intake intake;
    protected final Hood hood;
    protected final Flap flap;
    protected final PhotonVisionModule visionModule;

    public SaarIsAutonomous(SwerveDrive swerveDrive, Shooter shooter, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule, String initialPathPath) {
        this.swerveDrive = swerveDrive;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.intake = intake;
        this.hood = hood;
        this.flap = flap;
        this.visionModule = visionModule;

        var initialState = PathPlanner.loadPath(initialPathPath, Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL).getInitialState();
        addCommands(new InstantCommand(() -> {
            swerveDrive.resetOdometry(initialState.poseMeters, initialState.holonomicRotation);
            Robot.resetAngle(initialState.holonomicRotation);
        }));
    }

    protected FollowPath followPath(String path) {
        return new FollowPath(
                PathPlanner.loadPath(path, Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL),
                swerveDrive::getPose,
                swerveDrive.getKinematics(),
                new PIDController(Constants.Autonomous.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Autonomous.KP_Y_CONTROLLER, 0, 0),
                swerveDrive::setStates,
                swerveDrive);
    }

    protected CommandBase followPathAndPickup(String path) {
        return new ParallelRaceGroup(
                followPath(path),
                pickup(10)
//                new RunCommand(() -> shooter.setVelocity(3400), shooter)
        );
    }

    protected CommandBase shootAndAdjust(double timeout) {
        Supplier<Pose2d> swervePose = swerveDrive::getPose;
        Supplier<Transform2d> poseRelativeToTarget = () -> Constants.Vision.HUB_POSE.minus(swervePose.get());
        DoubleSupplier distanceFromTarget = visionModule::getDistance;
        DoubleSupplier yaw = () -> visionModule.getYaw().orElse(Robot.getAngle().minus(new Rotation2d(
                        Math.atan2(
                                poseRelativeToTarget.get().getY(),
                                poseRelativeToTarget.get().getX()
                        )
                )
        ).getDegrees());

        return new SequentialCommandGroup(
                new AdjustToTargetOnCommand(swerveDrive, yaw, visionModule::hasTargets).withTimeout(0.3),
                new ParallelRaceGroup(
                        new BackAndShootCargo(
                                shooter,
                                hood,
                                conveyor,
                                flap,
                                distanceFromTarget
                        ).withTimeout(timeout),
                        new IntakeCargo(intake, Constants.Intake.DEFAULT_POWER::get),
                        new AdjustToTargetOnCommand(swerveDrive, yaw, visionModule::hasTargets)
                ));
    }

    protected CommandBase shoot(double timeout) {
        DoubleSupplier distanceFromTarget = visionModule::getDistance;
        DoubleSupplier conveyorPower = Constants.Conveyor.DEFAULT_POWER::get;

        return new SequentialCommandGroup(
                new ParallelRaceGroup(new BackAndShootCargo(
                        shooter,
                        hood,
                        conveyor,
                        flap,
                        distanceFromTarget)
                        .withTimeout(timeout),
                        new IntakeCargo(intake, Constants.Intake.DEFAULT_POWER::get)
                ));
    }


    protected CommandBase pickup(double timeout) {
        return new PickUpCargo(
                conveyor,
                flap,
                intake,
                Constants.Conveyor.DEFAULT_POWER.get(),
                () -> Utils.map(MathUtil.clamp(Math.hypot(swerveDrive.getChassisSpeeds().vxMetersPerSecond, swerveDrive.getChassisSpeeds().vyMetersPerSecond), 0, 4), 0, 4, 0.4, 0.25)
        ).withTimeout(timeout);
    }

    protected CommandBase turnToAngle(Supplier<Rotation2d> target) {
        return new TurnToAngle(
                swerveDrive,
                target
        );
    }

    protected CommandBase reachVelocityByDistance(double distance) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setVelocity(Shoot.getSetpointVelocity(distance))),
                new InstantCommand(() -> hood.setSolenoid(distance < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance)));
    }

    protected CommandBase reachVelocity(double velocity, double distance) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setVelocity(velocity)),
                new InstantCommand(() -> hood.setSolenoid(distance < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance)));
    }

    protected CommandBase confirmShooting() {
        return new ConveyCargo(conveyor);
    }

    protected CommandBase confirmShootingSlower() {
        return new OldConveyNoWait(conveyor, () -> !conveyor.isPreFlapBeamConnected());
    }


}
