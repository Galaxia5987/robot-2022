package frc.robot.subsystems.drivetrain.commands.testing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveDirection extends SequentialCommandGroup {


    public DriveDirection(SwerveDrive swerveDrive) {
        addCommands(
                new RunCommand(() -> swerveDrive.setPower(1))
                        .withTimeout(5),
                new RunCommand(() -> swerveDrive.setPower(-1))
                        .withTimeout(5),
                new RunCommand(() -> swerveDrive.holonomicDrive(0, 1, 0))
                        .withTimeout(5),
                new RunCommand(() -> swerveDrive.holonomicDrive(0, -1, 0))
                        .withTimeout(5)

        );


    }


}