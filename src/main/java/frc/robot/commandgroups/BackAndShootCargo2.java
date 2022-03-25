package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.HoodCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import webapp.FireLog;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class BackAndShootCargo2 extends SequentialCommandGroup {

    class ConveyToShooter2 extends CommandBase {
        private final Conveyor conveyor;
        private final BooleanSupplier preFlapSupplier;
        private final Timer timer = new Timer();
        private final DoubleSupplier velocitySupplier;
        private final Timer delayTimer = new Timer();
        private boolean last = false;
        private boolean getBallToPreFlap = true;
        private boolean wait = true;

        public ConveyToShooter2(Conveyor conveyor, BooleanSupplier preFlapSupplier, DoubleSupplier velocitySupplier) {
            this.conveyor = conveyor;
            this.preFlapSupplier = preFlapSupplier;
            this.velocitySupplier = velocitySupplier;
            addRequirements(conveyor);
        }

        @Override
        public void initialize() {
            timer.stop();
            delayTimer.stop();
            if (preFlapSupplier.getAsBoolean()) {
                getBallToPreFlap = false;
            }
            wait = true;
            last = false;
        }

        @Override
        public void execute() {
            FireLog.log("Shooter velocity", velocitySupplier.getAsDouble());
            if (RobotContainer.hardCodedVelocity) {
                FireLog.log("Shooter setpoint", RobotContainer.hardCodedVelocityValue);
            } else {
                if (RobotContainer.cachedHasTarget) {
                    FireLog.log("Shooter setpoint", RobotContainer.cachedSetpointForShooter);
                } else {
                    FireLog.log("Shooter setpoint", RobotContainer.odometryCachedSetpoint);
                }
            }

            if (getBallToPreFlap) {
                conveyor.setPower(Constants.Conveyor.SHOOT_POWER);
            } else {
                conveyor.setPower(0);
            }

            if (preFlapSupplier.getAsBoolean()) {
                if (!last) {
                    last = true;
                    getBallToPreFlap = false;
                    timer.start();
                    timer.reset();
                    delayTimer.start();
                    delayTimer.reset();
                }
            } else {
                last = false;
                getBallToPreFlap = true;
            }

            if (timer.hasElapsed(0.3)) {
                getBallToPreFlap = true;
                SmartDashboard.putNumber("time", timer.get());
                timer.reset();
                timer.stop();
            }
        }

        @Override
        public void end(boolean interrupted) {
            timer.stop();
            delayTimer.stop();
            conveyor.setPower(0);
        }

        @Override
        public boolean isFinished() {
            return super.isFinished();
        }
    }
    public BackAndShootCargo2(Shooter shooter, Hood hood, Conveyor conveyor, Flap flap, DoubleSupplier distanceFromTarget) {
        addCommands(new InstantCommand(() -> RobotContainer.cachedSetpointForShooter = RobotContainer.setpointSupplierForShooterFromVision.getAsDouble()));
        addCommands(new InstantCommand(() -> {
                RobotContainer.cachedDistanceForHood = RobotContainer.distanceSupplierFromVision.getAsDouble() - 0.5;

//            if (RobotContainer.distanceSupplier.getAsDouble() > 3.4 && RobotContainer.distanceSupplier.getAsDouble() < 4.5)
//                RobotContainer.cachedDistance = RobotContainer.distanceSupplier.getAsDouble() - 0.3;
//            else RobotContainer.cachedDistance = RobotContainer.distanceSupplier.getAsDouble();
        }));
        addCommands(new InstantCommand(() -> RobotContainer.odometryCachedSetpoint = RobotContainer.odometrySetpointSupplier.getAsDouble()));
        addCommands(new InstantCommand(() -> RobotContainer.odometryCachedDistance = RobotContainer.odometryDistanceSupplier.getAsDouble()));
        addCommands(new InstantCommand(() -> RobotContainer.cachedHasTarget = !RobotContainer.playWithoutVision && RobotContainer.hasTarget.getAsBoolean()));
        addCommands(new InstantCommand(() -> RobotContainer.shooting = true));

        addCommands(new Convey(conveyor, -0.25).withTimeout(0.075),
                new ParallelCommandGroup(
                                new HoodCommand(hood),
                                new ConveyToShooter2(conveyor, () -> !conveyor.isPreFlapBeamConnected(), shooter::getVelocity),
                                new InstantCommand(flap::allowShooting),
                                new Shoot(shooter, hood, distanceFromTarget)
                ));

    }
}
