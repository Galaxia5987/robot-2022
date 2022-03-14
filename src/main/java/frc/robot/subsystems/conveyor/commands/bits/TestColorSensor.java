package frc.robot.subsystems.conveyor.commands.bits;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;

public class TestColorSensor extends SequentialCommandGroup {

    public TestColorSensor(Conveyor conveyor) {
        addCommands(
                new RunCommand(() -> System.out.println("Enter blue cargo")).withTimeout(4),
                new RunCommand(() -> checkColorBlue(conveyor)).withTimeout(10),
                new RunCommand(() -> System.out.println("Enter red cargo")).withTimeout(4),
                new RunCommand(() -> checkColorRed(conveyor)).withTimeout(10)
        );
    }

    public static void checkColorBlue(Conveyor conveyor) {
        if (conveyor.getColor() != DriverStation.Alliance.Blue) {
            System.out.println("Waiting for blue cargo....");
        } else System.out.println("Sensed blue!!!");
    }

    public static void checkColorRed(Conveyor conveyor) {
        if (conveyor.getColor() != DriverStation.Alliance.Red) {
            System.out.println("Waiting for red cargo....");
        } else System.out.println("Sensed red!!!");
    }
}