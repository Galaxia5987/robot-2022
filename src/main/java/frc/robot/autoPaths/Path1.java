package frc.robot.autoPaths;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commandgroups.PickUpCargo;
import frc.robot.commandgroups.ShootCargo;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;


public class Path1 extends SequentialCommandGroup {
    private DoubleSupplier distanceFromTarget;
    private DoubleSupplier conveyorPower;

    public Path1(Shooter shooter, SwerveDrive drivetrain, Conveyor conveyor, Intake intake, Hood hood, Flap flap) {
        var path0 = PathPlanner.loadPath("p0 - Taxi from up low tarmac(1.1)", Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL);

        addCommands(new PickUpCargo(conveyor, intake, ));

        var path1 = PathPlanner.loadPath("p0 - Go to middle tarmac(1.2.1)", Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL);

        addCommands(new ShootCargo(shooter,
                hood,
                conveyor,
                flap,
                distanceFromTarget,
                () -> Constants.Conveyor.DEFAULT_POWER));

    }
}



