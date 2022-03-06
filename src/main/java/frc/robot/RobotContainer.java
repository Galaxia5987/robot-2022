package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autoPaths.FourCargoAuto;
import frc.robot.commandgroups.Outtake;
import frc.robot.commandgroups.PickUpCargo;
import frc.robot.commandgroups.bits.RunAllBits;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveAndAdjustWithVision;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.helicopter.Helicopter;
import frc.robot.subsystems.helicopter.commands.JoystickPowerHelicopter;
import frc.robot.subsystems.helicopter.commands.MoveHelicopter;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.BackAndShootCargo;
import frc.robot.utils.LedSubsystem;
import frc.robot.utils.PhotonVisionModule;
import webapp.Webserver;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RobotContainer {
    private static final Joystick joystick = new Joystick(Ports.Controls.JOYSTICK);
    private static final Joystick joystick2 = new Joystick(Ports.Controls.JOYSTICK2);
    public static LedSubsystem ledSubsystem = new LedSubsystem();
    // The robot's subsystems and commands are defined here...
    final PhotonVisionModule photonVisionModule = new PhotonVisionModule("photonvision", null);
    private final XboxController xbox = new XboxController(Ports.Controls.XBOX);
    private final JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xbox, XboxController.Button.kB.value);
    private final JoystickButton x = new JoystickButton(xbox, XboxController.Button.kX.value);
    private final JoystickButton y = new JoystickButton(xbox, XboxController.Button.kY.value);
    private final JoystickButton lb = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rb = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
    private final JoystickButton start = new JoystickButton(xbox, XboxController.Button.kStart.value);
    private final JoystickButton back = new JoystickButton(xbox, XboxController.Button.kBack.value);
    private final Trigger rt = new Trigger(() -> xbox.getRightTriggerAxis() > Constants.Control.RIGHT_TRIGGER_DEADBAND);
    private final Trigger lt = new Trigger(() -> xbox.getLeftTriggerAxis() > Constants.Control.RIGHT_TRIGGER_DEADBAND);
    private final Trigger upPov = new Trigger(() -> xbox.getPOV() == 0);
    private final Trigger downPov = new Trigger(() -> xbox.getPOV() == 180);
    private final Trigger rightPov = new Trigger(() -> xbox.getPOV() == 90);
    private final Trigger leftPov = new Trigger(() -> xbox.getPOV() == 270);
    private final JoystickButton leftTrigger = new JoystickButton(joystick, Joystick.ButtonType.kTrigger.value);
    private final JoystickButton rightTrigger = new JoystickButton(joystick2, Joystick.ButtonType.kTrigger.value);
    private final JoystickButton two = new JoystickButton(joystick, 2);
    private final JoystickButton twoJoystick2 = new JoystickButton(joystick2, 2);
    private final JoystickButton twelve = new JoystickButton(joystick, 12);
    private final SwerveDrive swerve = SwerveDrive.getFieldOrientedInstance(photonVisionModule::estimatePose);
    private final Intake intake = Intake.getInstance();
    private final Conveyor conveyor = Conveyor.getInstance();
    private final Flap flap = Flap.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();
    private final Helicopter helicopter = Helicopter.getInstance();
    private double speedMultiplier = 1;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings and default commands
        configureDefaultCommands();
        if (Robot.debug) {
            startFireLog();
        }

        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        Supplier<Pose2d> swervePose = swerve::getPose;
        Supplier<Transform2d> poseRelativeToTarget = () -> Constants.Vision.HUB_POSE.minus(swervePose.get());
        DoubleSupplier yaw = () -> photonVisionModule.getYaw().orElse(Robot.getAngle().minus(new Rotation2d(
                        Math.atan2(
                                poseRelativeToTarget.get().getY(),
                                poseRelativeToTarget.get().getX()
                        )
                )
        ).getDegrees());
        swerve.setDefaultCommand(
                new DriveAndAdjustWithVision(
                        swerve,
                        () -> -joystick.getY() * speedMultiplier,
                        () -> -joystick.getX() * speedMultiplier,
                        () -> -joystick2.getX() * speedMultiplier,
                        yaw,
                        rightTrigger::get,
                        photonVisionModule::getDistance
                )
        );
        helicopter.setDefaultCommand(new JoystickPowerHelicopter(helicopter, xbox::getLeftY));
//        shooter.setDefaultCommand(new Shoot(shooter, hood, WebConstant.of("Shooter", "Setpoint", 0)::get, () -> true));
    }

    private void configureButtonBindings() {
        Supplier<Pose2d> swervePose = swerve::getPose;
        Supplier<Transform2d> poseRelativeToTarget = () -> Constants.Vision.HUB_POSE.minus(swervePose.get());

        DoubleSupplier distanceFromTarget = () -> photonVisionModule.hasTargets() ?
                photonVisionModule.getDistance() :
                Math.hypot(poseRelativeToTarget.get().getX(), poseRelativeToTarget.get().getY());

//        a.whileHeld(new IntakeCargo(intake, () -> -Constants.Intake.DEFAULT_POWER.get()));
        b.whileHeld(new Convey(conveyor, Constants.Conveyor.DEFAULT_POWER.get()));
        leftPov.whileActiveOnce(new InstantCommand(hood::toggle));
        x.whenPressed(intake::toggleRetractor);
        back.whenPressed(flap::toggleFlap);
//        upPov.whileActiveOnce(new InstantCommand(hood::toggle));
        rightPov.whileActiveOnce(new InstantCommand(helicopter::toggleStopper));
        upPov.and(start).whileActiveOnce(new MoveHelicopter(helicopter, Constants.Helicopter.SECOND_RUNG));
        downPov.and(start).whileActiveOnce(new MoveHelicopter(helicopter, 0));
//        rt.whileActiveContinuous(new Shoot(shooter, hood, WebConstant.of("Shooter", "Setpoint", 0)::get, () -> true));

        rt.whileActiveContinuous(new BackAndShootCargo(
                shooter, hood, conveyor, flap,
                () -> Constants.Conveyor.SHOOT_POWER,
                distanceFromTarget));
        lt.whileActiveContinuous(new PickUpCargo(conveyor, flap, intake, Constants.Conveyor.DEFAULT_POWER.get(), Constants.Intake.DEFAULT_POWER::get));
        lb.whileHeld(new Outtake(intake, conveyor, flap, shooter, hood, () -> false));
        rb.whileHeld(new Convey(conveyor, -Constants.Conveyor.DEFAULT_POWER.get()));
        start.whenPressed(photonVisionModule::toggleLeds);
        y.whenPressed(new RunCommand(() -> shooter.setVelocity(3350), shooter).withInterrupt(rt::get));
//        twelve.whenPressed(() -> swerve.resetPoseEstimator(new Pose2d(7, 5, new Rotation2d())));


        leftTrigger.whenPressed(() -> speedMultiplier = (speedMultiplier == 0.5 ? 1 : 0.5));
        two.whenPressed((Runnable) Robot::resetAngle);
        twoJoystick2.whileHeld(new TurnToAngle(swerve, () -> 0));


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
//        return new RunAllBits(swerve, shooter, conveyor, intake, flap, hood);
        return new FourCargoAuto(shooter, swerve, conveyor, intake, hood, flap, photonVisionModule);
    }

    /**
     * Initiates the value tuner.
     * <p>
     * Initiates the port of team 225s Fire-Logger.
     */
    private void startFireLog() {
        try {
            new Webserver();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
