package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.FiveCargoAuto;
import frc.robot.commandgroups.*;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveAndAdjustWithVision;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.helicopter.Helicopter;
import frc.robot.subsystems.helicopter.commands.JoystickPowerHelicopter;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.BackAndShootCargo;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.LedSubsystem;
import frc.robot.utils.PhotonVisionModule;
import frc.robot.utils.Utils;
import webapp.Webserver;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RobotContainer {
    public static final boolean playWithoutVision = true;
    public static boolean hardCodedVelocity = true;
    public static double hardCodedDistance = 3.8;
    public static double hardCodedVelocityValue = Shoot.getSetpointVelocity(hardCodedDistance);

    // The robot's subsystems and commands are defined here...
//    public static SnakeSubsystem snakeSubsystem = new SnakeSubsystem();
    public static DoubleSupplier setpointSupplier;
    public static DoubleSupplier distanceSupplier;
    public static DoubleSupplier odometrySetpointSupplier;
    public static DoubleSupplier odometryDistanceSupplier;
    public static double cachedSetpoint = 0;
    public static double cachedDistance = 0;
    public static double odometryCachedSetpoint = 0;
    public static double odometryCachedDistance = 0;
    public static boolean cachedHasTarget = true;

    public static BooleanSupplier hasTarget;
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
        setpointSupplier = () -> Shoot.getSetpointVelocity(photonVisionModule.getDistance());
        distanceSupplier = photonVisionModule::getDistance;
        odometrySetpointSupplier = () -> Shoot.getSetpointVelocity(swerve.getOdometryDistance());
        odometryDistanceSupplier = swerve::getOdometryDistance;
        hasTarget = photonVisionModule::hasTargets;

        autonomousCommand = new FiveCargoAuto(shooter, swerve, conveyor, intake, hood, flap, photonVisionModule);
//        autonomousCommand = new Yoni(shooter, swerve, conveyor, intake, hood, flap, photonVisionModule);
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
            Xbox.upPov.whileActiveOnce(new SafetyShootCargo(shooter, conveyor, flap, hood, distanceFromTarget));
            Xbox.downPov.whileActiveOnce(new LowGoalShot(shooter, conveyor, flap, hood));

            Xbox.rt.whileActiveContinuous(new BackAndShootCargo(
                    shooter, hood, conveyor, flap,
                    () -> Constants.Conveyor.SHOOT_POWER,
                    () -> 0));
//            Xbox.lt.whileActiveContinuous(new PickUpCargo(conveyor, flap, intake, 0.7,
//                    () -> Utils.map(MathUtil.clamp(Math.hypot(swerve.getChassisSpeeds().vxMetersPerSecond, swerve.getChassisSpeeds().vyMetersPerSecond), 0, 4), 0, 4, 0.7, 0.4)));
            Xbox.lt.whileActiveContinuous(new SmartIndexing(shooter, conveyor, intake, flap, () -> Utils.map(MathUtil.clamp(Math.hypot(swerve.getChassisSpeeds().vxMetersPerSecond, swerve.getChassisSpeeds().vyMetersPerSecond), 0, 4), 0, 4, 0.7, 0.4)));
            Xbox.lb.whileHeld(new Outtake(intake, conveyor, flap, shooter, hood, () -> false));
            Xbox.rb.whileHeld(new Convey(conveyor, -Constants.Conveyor.SHOOT_POWER));

            Xbox.start.whenPressed(photonVisionModule::toggleLeds);
            Xbox.back.whenPressed(new OneBallOuttake(intake, conveyor, () -> conveyor.getColorSensorProximity() >= 150));

            Xbox.rightJoystickButton
                    .whileHeld(() -> hardCodedVelocity = true)
                    .whenReleased(() -> hardCodedVelocity = false);

        }

        { // Joystick button bindings.
            Joysticks.leftTrigger.whenPressed(() -> {
                speedMultiplier = (speedMultiplier == 0.5 ? 1 : 0.5);
                thetaMultiplier = 1.5 * speedMultiplier;
            });

            Joysticks.leftTwo.whenPressed((Runnable) Robot::resetAngle);
            Joysticks.rightTwo.whileHeld(new TurnToAngle(swerve, () -> new Rotation2d()));
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

        public static final JoystickButton rightJoystickButton = new JoystickButton(controller, XboxController.Button.kRightStick.value);
    }
}
