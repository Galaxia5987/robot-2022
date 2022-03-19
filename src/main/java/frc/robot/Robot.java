// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.intake.Intake;
import frc.robot.valuetuner.NetworkTableConstant;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static final boolean debug = true;//!DriverStation.isFMSAttached();
    public static final AHRS navx = new AHRS(SPI.Port.kMXP);
    //    private final AddressableLED led = new AddressableLED(1);
    private static final Rotation2d startAngle = new Rotation2d();
    private static Rotation2d zeroAngle = new Rotation2d();
    public PowerDistribution pdp = new PowerDistribution();
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
//    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(54);

    /**
     * Gets the current angle of the robot in respect to the start angle.
     *
     * @return the current angle of the robot in respect to the start angle.
     */
    public static Rotation2d getAngle() {
        return getRawAngle().minus(zeroAngle);
    }

    /**
     * Gets the raw angle from the navx.
     *
     * @return the angle of the robot in respect to the angle of the robot initiation time.
     */
    public static Rotation2d getRawAngle() {
        return Robot.navx.getRotation2d();
    }

    /**
     * Resets the angle of the navx to the current angle.
     */
    public static void resetAngle() {
        resetAngle(new Rotation2d());
    }

    /**
     * Resets the angle of the navx to the current angle.
     *
     * @param angle the angle in -180 to 180 degrees coordinate system.
     */
    public static void resetAngle(Rotation2d angle) {
        zeroAngle = getRawAngle().minus(angle);
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

//        startAngle = Robot.navx.getRotation2d();
        resetAngle();
        if (debug) {
            NetworkTableConstant.initializeAllConstants();
        }
        m_robotContainer = new RobotContainer();
        UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
        MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
        mjpegServer1.setSource(usbCamera);

//        led.setLength(buffer.getLength());
//        led.start();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
//        for (int i = 0; i < buffer.getLength(); i++) {
//            buffer.setLED(i, Color.kPurple);
//        }
//        led.setData(buffer);
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
        Intake.getInstance().closeRetractor();
        Flap.getInstance().blockShooter();
        m_robotContainer.photonVisionModule.setLeds(true);
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
