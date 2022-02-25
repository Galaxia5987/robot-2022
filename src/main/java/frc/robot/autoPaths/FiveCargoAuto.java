package frc.robot.autoPaths;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import java.util.function.Function;

public class FiveCargoAuto extends SequentialCommandGroup {

    private DoubleSupplier distanceFromTarget;
    private DoubleSupplier conveyorPower;

    /* Taxi from low right tarmac, pickup low cargo, shoot, pickup middle cargo,
     go to terminal, pickup cargo from terminal, go to shooting position, shoot, pickup up cargo,
     park near up tarmac, shoot.(10)
     */
    public FiveCargoAuto(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap) {

        var rotationPID = new ProfiledPIDController(Constants.Autonomous.KP_THETA_CONTROLLER, 0, 0, new TrapezoidProfile.Constraints(Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL));
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        Function<String, PPSwerveControllerCommand> createCommand = path -> new PPSwerveControllerCommand(
                PathPlanner.loadPath(path, Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL),
                swerveDrive::getPose,
                swerveDrive.getKinematics(),
                new PIDController(Constants.Autonomous.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.Autonomous.KP_Y_CONTROLLER, 0, 0),
                rotationPID,
                swerveDrive::setStates,
                swerveDrive);

       addCommands(new ParallelCommandGroup((createCommand.apply("p4 - Taxi from low right tarmac and pickup low cargo(10.1)")),
                new PickUpCargo(
                        conveyor,
                        flap,
                        intake,
                        Constants.Conveyor.DEFAULT_POWER.get(),
                        Constants.Intake.DEFAULT_POWER::get
                ).withTimeout(3)));

        addCommands(new ShootCargo(
                shooter,
                hood,
                conveyor,
                flap,
                conveyorPower,
                distanceFromTarget)
                .withTimeout(3));

        addCommands(new ParallelCommandGroup((createCommand.apply("p4 - Pickup middle cargo(10.2)")),
                new PickUpCargo(
                        conveyor,
                        flap,
                        intake,
                        Constants.Conveyor.DEFAULT_POWER.get(),
                        Constants.Intake.DEFAULT_POWER::get
                ).withTimeout(3)));

        addCommands((createCommand.apply("p3 - Going to terminal(9.3)")));

        addCommands(new PickUpCargo(
                conveyor,
                flap,
                intake,
                Constants.Conveyor.DEFAULT_POWER.get(),
                Constants.Intake.DEFAULT_POWER::get
        ).withTimeout(3));


        addCommands(createCommand.apply("p4 - Shooting position(10.4)"));

        addCommands(new ShootCargo(
                shooter,
                hood,
                conveyor,
                flap,
                conveyorPower,
                distanceFromTarget)
                .withTimeout(3));


        addCommands(createCommand.apply("p4 - Pickup up cargo(10.5)"));

        addCommands(createCommand.apply("p4 - Going to up tarmac(10.6)"));

        addCommands(new ShootCargo(
                shooter,
                hood,
                conveyor,
                flap,
                conveyorPower,
                distanceFromTarget)
                .withTimeout(3));
    }
}