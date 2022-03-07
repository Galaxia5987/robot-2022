package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

import java.util.function.DoubleSupplier;


public class TaxiFromUpLowAndShoot extends SaarIsAutonomous {
    private DoubleSupplier distanceFromTarget;
    private DoubleSupplier conveyorPower;

    // Taxi from up low tarmac, shoot pre-loaded cargo, park near up tarmac.(1)
    public TaxiFromUpLowAndShoot(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "p0 - Taxi from up low tarmac(1.1)");
        addCommands(new ParallelRaceGroup(followPath("p0 - Taxi from up low tarmac(1.1)"), new RunCommand(() -> shooter.setVelocity(3300), shooter)));
        addCommands(shootAndAdjust(3));

//        addCommands(followPath("p0 - Go to up tarmac(1.2.2)"));
    }
}