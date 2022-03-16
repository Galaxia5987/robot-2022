package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.TaxiFromLowRightPickShootPickShoot;
import frc.robot.commandgroups.BackAndShootCargoSort;
import frc.robot.commandgroups.OneBallOuttake;
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
    public static final boolean playWithoutVision = false;
    public static final boolean hardCodedVelocity = true;

    // The robot's subsystems and commands are defined here...
    public static LedSubsystem ledSubsystem = new LedSubsystem();
    final PhotonVisionModule photonVisionModule = new PhotonVisionModule("photonvision", null);
    private final SwerveDrive swerve = SwerveDrive.getFieldOrientedInstance(photonVisionModule::estimatePose);
    private final Intake intake = Intake.getInstance();
    private final Conveyor conveyor = Conveyor.getInstance();
    private final Flap flap = Flap.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();
    private final Helicopter helicopter = Helicopter.getInstance();
    private final CommandBase autonomousCommand;
    private double speedMultiplier = 1;
    private double thetaMultiplier = 1.5;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        autonomousCommand = new TaxiFromLowRightPickShootPickShoot(shooter, swerve, conveyor, intake, hood, flap, photonVisionModule);
        // Configure the button bindings and default commands
        configureDefaultCommands();
        if (Robot.debug) {
            startFireLog();
        }

        configureButtonBindings();

    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                new DriveAndAdjustWithVision(
                        swerve,
                        () -> -Joysticks.leftJoystick.getY() * speedMultiplier,
                        () -> -Joysticks.leftJoystick.getX() * speedMultiplier,
                        () -> -Joysticks.rightJoystick.getX() * thetaMultiplier,
                        () -> photonVisionModule.getYaw().orElse(0),
                        Joysticks.rightTrigger::get,
                        photonVisionModule::getDistance,
                        photonVisionModule::hasTargets)
        );
        helicopter.setDefaultCommand(new JoystickPowerHelicopter(helicopter, Xbox.controller::getLeftY));
    }

    private void configureButtonBindings() {
        Supplier<Pose2d> swervePose = swerve::getPose;
        Supplier<Transform2d> poseRelativeToTarget = () -> Constants.Vision.HUB_POSE.minus(swervePose.get());
        DoubleSupplier distanceFromTarget = () -> photonVisionModule.hasTargets() ?
                photonVisionModule.getDistance() :
                Math.hypot(poseRelativeToTarget.get().getX(), poseRelativeToTarget.get().getY());

        { // Xbox controller button bindings.
            Xbox.b.whileHeld(new ParallelCommandGroup(
                    new InstantCommand(flap::allowShooting),
                    new Convey(conveyor, Constants.Conveyor.DEFAULT_POWER.get())
            ));
            Xbox.x.whenPressed(intake::toggleRetractor);
            Xbox.y.whenPressed(new RunCommand(() -> {
                shooter.setVelocity(3350);
            }, shooter).withInterrupt(Xbox.rt::get));
            Xbox.a.whileHeld(new BackAndShootCargoSort(shooter, hood, conveyor, flap,
                    () -> Constants.Conveyor.SHOOT_POWER,
                    distanceFromTarget, photonVisionModule::hasTargets, swerve::getOdometryDistance));

            Xbox.leftPov.whileActiveOnce(new InstantCommand(hood::toggle));
            Xbox.rightPov.whileActiveOnce(new InstantCommand(helicopter::toggleStopper));
            Xbox.upPov.and(Xbox.start).whileActiveOnce(new MoveHelicopter(helicopter, Constants.Helicopter.SECOND_RUNG));
            Xbox.downPov.and(Xbox.start).whileActiveOnce(new MoveHelicopter(helicopter, 0));

            Xbox.rt.whileActiveContinuous(new BackAndShootCargo(
                    shooter, hood, conveyor, flap,
                    () -> Constants.Conveyor.SHOOT_POWER,
                    () -> 0, photonVisionModule::hasTargets, swerve::getOdometryDistance));
            Xbox.lt.whileActiveContinuous(new PickUpCargo(conveyor, flap, intake, Constants.Conveyor.DEFAULT_POWER.get(), Constants.Intake.DEFAULT_POWER::get));
            Xbox.lb.whileHeld(new Outtake(intake, conveyor, flap, shooter, hood, () -> false));
            Xbox.rb.whileHeld(new Convey(conveyor, -Constants.Conveyor.SHOOT_POWER));

            Xbox.start.whenPressed(photonVisionModule::toggleLeds);
            Xbox.back.whenPressed(new OneBallOuttake(intake, conveyor, () -> conveyor.getColorSensorProximity() >= 150));
        }

        { // Joystick button bindings.
            Joysticks.leftTrigger.whenPressed(() -> {
                speedMultiplier = (speedMultiplier == 0.5 ? 1 : 0.5);
                thetaMultiplier = 1.5 * speedMultiplier;
            });

            Joysticks.leftTwo.whenPressed((Runnable) Robot::resetAngle);
            Joysticks.rightTwo.whileHeld(new TurnToAngle(swerve, () -> 0));
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
//        return new RunAllBits(swerve, shooter, conveyor, intake, flap, hood, helicopter);
        return autonomousCommand;
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

    private static final class Joysticks {
        public static final Joystick leftJoystick = new Joystick(Ports.Controls.JOYSTICK);
        public static final Joystick rightJoystick = new Joystick(Ports.Controls.JOYSTICK2);

        public static final JoystickButton leftTrigger = new JoystickButton(leftJoystick, Joystick.ButtonType.kTrigger.value);
        public static final JoystickButton rightTrigger = new JoystickButton(rightJoystick, Joystick.ButtonType.kTrigger.value);

        public static final JoystickButton leftTwo = new JoystickButton(leftJoystick, 2);
        public static final JoystickButton rightTwo = new JoystickButton(rightJoystick, 2);
    }

    private static final class Xbox {
        public static final XboxController controller = new XboxController(Ports.Controls.XBOX);

        public static final JoystickButton a = new JoystickButton(controller, XboxController.Button.kA.value);
        public static final JoystickButton b = new JoystickButton(controller, XboxController.Button.kB.value);
        public static final JoystickButton x = new JoystickButton(controller, XboxController.Button.kX.value);
        public static final JoystickButton y = new JoystickButton(controller, XboxController.Button.kY.value);

        public static final JoystickButton lb = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        public static final JoystickButton rb = new JoystickButton(controller, XboxController.Button.kRightBumper.value);

        public static final JoystickButton start = new JoystickButton(controller, XboxController.Button.kStart.value);
        public static final JoystickButton back = new JoystickButton(controller, XboxController.Button.kBack.value);

        public static final Trigger rt = new Trigger(() -> controller.getRightTriggerAxis() > Constants.Control.RIGHT_TRIGGER_DEADBAND);
        public static final Trigger lt = new Trigger(() -> controller.getLeftTriggerAxis() > Constants.Control.LEFT_TRIGGER_DEADBAND);

        public static final Trigger upPov = new Trigger(() -> controller.getPOV() == 0);
        public static final Trigger downPov = new Trigger(() -> controller.getPOV() == 180);
        public static final Trigger rightPov = new Trigger(() -> controller.getPOV() == 90);
        public static final Trigger leftPov = new Trigger(() -> controller.getPOV() == 270);
    }
}
