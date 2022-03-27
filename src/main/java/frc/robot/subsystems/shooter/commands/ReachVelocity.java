package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.Shooter;

public class ReachVelocity extends CommandBase {
    private final Shooter shooter;

    public ReachVelocity(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (RobotContainer.warmUpShooting) {
            if (RobotContainer.hardCodedVelocity) {
                shooter.setVelocity(RobotContainer.hardCodedVelocityValue);
                SmartDashboard.putString("speed_state", Math.abs(shooter.getVelocity() - RobotContainer.hardCodedVelocityValue) <= 50 ? "green" : Math.abs(shooter.getVelocity() - RobotContainer.hardCodedVelocityValue) <= 100 ? "yellow" : "red");
            } else {
                if (RobotContainer.smartWarmUp) {
                    if (RobotContainer.hasTarget.getAsBoolean() && !RobotContainer.playWithoutVision && !DriverStation.isAutonomous()) {
                        if (Math.abs(RobotContainer.Suppliers.yawSupplier.getAsDouble()) < 10) { // with vision
                            shooter.setVelocity(RobotContainer.setpointVelocity);
                        }
                    } else { // with odometry
                        shooter.setVelocity(RobotContainer.setpointVelocity - 300);
                    }
                } else {
                    if (RobotContainer.shooting) {
                        shooter.setVelocity(RobotContainer.setpointVelocity);
                    } else {
                        shooter.setVelocity(3530);
                    }
                }
            }
        } else {
            shooter.setPower(0);
        }
    }
}
