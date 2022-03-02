package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.HolonomicDrive;
import frc.robot.subsystems.drivetrain.commands.testing.TurnAllModuleMotors;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeCargo;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.subsystems.shooter.commands.WarmUpShooter;

import java.util.Arrays;

public class TurnAllMotors extends ParallelCommandGroup {
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final SwerveDrive swerve;
    private final Intake intake;
    private final Timer timer = new Timer();

    public TurnAllMotors(Shooter shooter,
                         Conveyor conveyor,
                         SwerveDrive swerve,
                         Intake intake) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.swerve = swerve;
        this.intake = intake;

        addCommands(
                new WarmUpShooter(shooter, () -> 3),
                new Convey(conveyor, 0.5),
                new TurnAllModuleMotors(swerve),
                new IntakeCargo(intake, () -> 0.5)
        );
    }

    @Override
    public void execute() {
        super.execute();

        double[] swerveStates = new double[]{swerve.getModule(0).getVelocity(),
                                             swerve.getModule(1).getVelocity(),
                                             swerve.getModule(2).getVelocity(),
                                             swerve.getModule(3).getVelocity()};
        System.out.println(
                "        Shooter        |         " + "Conveyor         |         " + "Swerve          |         " + "Intake" + "\n" +
                "req = " + Shoot.getSetpointVelocity(3, false) + ", actual = " + shooter.getVelocity() + "|" +
                "req = 0.5, actual = " + conveyor.getPower() + "|" +
                "req = 0.5, actual = " + Arrays.toString(swerveStates) + "|" +
                "req = 0.5, actual = " + intake.getPower()
        );
    }
}
