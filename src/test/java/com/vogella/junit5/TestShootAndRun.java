package com.vogella.junit5;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
import frc.robot.commandgroups.ShootAndRun;

import java.util.function.DoubleSupplier;


public class TestShootAndRun {
    ShootAndRun shootAndRun
    Shooter shooter;
    SwerveDrive swerveDrive;
    Hood hood ;
    Conveyor conveyor;
    Flap flap;
    DoubleSupplier visionDistance;
    DoubleSupplier visionYaw;

    @BeforeEach
    void setUp(){
        Shooter.getInstance();
        SwerveDrive.getFieldOrientedInstance();
        Hood.getInstance();
        Conveyor.getInstance();
        Flap.getInstance();

        shootAndRun = new ShootAndRun(Shooter shooter, SwerveDrive swerveDrive, Hood hood, Conveyor conveyor, Flap flap, DoubleSupplier visionDistance, DoubleSupplier visionYaw);
    }
    @Test
    @DisplayName("Test Shoot and run functions")
    void testCalculateCurrentGoal(){
        assertEquals(new Translation2d(1, 0), shootAndRun.calculateCurrentGoal(1, 0),
                "Should give coordinates of (1, 0)");
    }
}
