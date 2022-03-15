package frc.robot.subsystems.conveyor.commands.bits;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;

public class TestColorSensor extends SequentialCommandGroup {

    public TestColorSensor(Conveyor conveyor, Intake intake) {
        addCommands(
                new RunCommand(intake::closeRetractor),
                new RunCommand(() -> System.out.println("Enter blue cargo")).withTimeout(4),
                new RunCommand(() -> checkColorBlue(conveyor)).withInterrupt(() -> conveyor.getColor() == DriverStation.Alliance.Blue),
                new RunCommand(() -> System.out.println("Enter red cargo")).withTimeout(4),
                new RunCommand(() -> checkColorRed(conveyor)).withInterrupt(() -> conveyor.getColor() == DriverStation.Alliance.Red),
                new RunCommand(() -> System.out.println("Sensor works!"))
        );
    }


    public static void checkColorBlue(Conveyor conveyor) {
        if (conveyor.getColor() != DriverStation.Alliance.Invalid) {
            System.out.println("Waiting for blue cargo....");
        } else if (conveyor.getColor() == DriverStation.Alliance.Red) {
            System.out.println("Error! sensed red");
        } else System.out.println("Sensed blue!!!");
    }

    public static void checkColorRed(Conveyor conveyor) {
        if (conveyor.getColor() != DriverStation.Alliance.Red) {
            System.out.println("Waiting for red cargo....");
        } else if (conveyor.getColor() == DriverStation.Alliance.Blue) {
            System.out.println("Error! sensed blue");
        } else System.out.println("Sensed red!!!");
    }
}