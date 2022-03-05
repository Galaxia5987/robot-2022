package frc.robot.autoPaths;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commandgroups.ShootCargo;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.auto.FollowPath;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class TaxiFromUpUpAndShoot extends SaarIsAutonomous {

    // Taxi from up up tarmac, shoot pre-loaded cargo, park near up tarmac.(2)
    public TaxiFromUpUpAndShoot(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "p0 - Taxi from up up tarmac(2.1)");

        addCommands(followPathAndPickup("p0 - Taxi from up up tarmac(2.1)"));

        addCommands(shootAndAdjust(3));

        addCommands(followPath("p0 - Go to up tarmac(1.2.2)"));
    }
}
