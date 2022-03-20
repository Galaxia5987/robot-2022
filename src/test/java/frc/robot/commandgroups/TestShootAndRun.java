package frc.robot.commandgroups;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.Utils;
import org.junit.Test;
import webapp.FireLog;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class TestShootAndRun {

    @Test
    public void testCalculateCurrentGoal() {
        //First quadrant
        assertEquals(new Translation2d(1, 0), ShootAndRun.calculateCurrentGoal(1, 0),
                "Should give coordinates of (1, 0)");
        assertEquals(new Translation2d(0, 1), ShootAndRun.calculateCurrentGoal(1, 90),
                "should give coordinates of (0,1)");
        assertEquals(new Translation2d(1, 1), ShootAndRun.calculateCurrentGoal(Math.sqrt(2), 45),
                "Should give coordinates of (1,1)");
        //Second quadrant
        assertEquals(new Translation2d(-1, 0), ShootAndRun.calculateCurrentGoal(1, 180),
                "should give coordinates of (-1, 0)");
        assertEquals(new Translation2d(-1, 1), ShootAndRun.calculateCurrentGoal(Math.sqrt(2), 135),
                "should give coordinates of (-1, 1)");
        //Third quadrant
        assertEquals(new Translation2d(-1, -1), ShootAndRun.calculateCurrentGoal(Math.sqrt(2), 225),
                "Should give coordinates of (-1 , -1)");
        //Fourth quadrant
        assertEquals(new Translation2d(1, -1), ShootAndRun.calculateCurrentGoal(Math.sqrt(2), 315),
                "should give coordinates of (-1, 0)");

    }

    @Test
    public void testCalculateVirtualGoal() {
// x - axis
        assertEquals(new Translation2d(8, 3), ShootAndRun.calculateVirtualGoal(
                        new Translation2d(8, 4), new ChassisSpeeds(0, 1, Math.toRadians(0)), 1),
                "should return coordinated of (8,3)");
        assertEquals(new Translation2d(8, 5), ShootAndRun.calculateVirtualGoal(
                        new Translation2d(8, 4), new ChassisSpeeds(0, -1, Math.toRadians(0)), 1),
                "should return coordinated of (8,2)");
// y - axis
        assertEquals(new Translation2d(7, 4), ShootAndRun.calculateVirtualGoal(
                        new Translation2d(8, 4), new ChassisSpeeds(1, 0, Math.toRadians(0)), 1),
                "should return coordinated of (7,4)");
        assertEquals(new Translation2d(9, 4), ShootAndRun.calculateVirtualGoal(
                        new Translation2d(8, 4), new ChassisSpeeds(-1, 0, Math.toRadians(0)), 1),
                "should return coordinated of (9,4)");
// Both
        assertEquals(new Translation2d(7 , 3), ShootAndRun.calculateVirtualGoal(
                        new Translation2d(8, 4), new ChassisSpeeds(1, 1, Math.toRadians(0)), 1),
                "should return coordinated of (7,3)");
        assertEquals(new Translation2d(9, 5), ShootAndRun.calculateVirtualGoal(
                        new Translation2d(8, 4), new ChassisSpeeds(-1, -1, Math.toRadians(0)), 1),
                "should return coordinated of (9,5)");
        assertEquals(new Translation2d(7, 5), ShootAndRun.calculateVirtualGoal(
                        new Translation2d(8, 4), new ChassisSpeeds(1, -1, Math.toRadians(0)), 1),
                "should return coordinated of (8,3)");
        assertEquals(new Translation2d(9, 3), ShootAndRun.calculateVirtualGoal(
                        new Translation2d(8, 4), new ChassisSpeeds(-1, 1, Math.toRadians(0)), 1),
                "should return coordinated of (8,3)");
    }
    @Test
    public void testGetYawToVirtualGoal(){
       // Movements within the first quadrant
        assertEquals(0, ShootAndRun.getYawToVirtualGoal(new Translation2d(1,1), 45),
                "The Robot needs to rotate 0 degrees");
        assertEquals(-45, ShootAndRun.getYawToVirtualGoal(new Translation2d(1,0), 45),
                "The Robot needs to rotate -45 degrees");
        assertEquals(90, ShootAndRun.getYawToVirtualGoal(new Translation2d(0,1), 0),
                "The Robot needs to rotate 90 degrees");
        assertEquals(-90, ShootAndRun.getYawToVirtualGoal(new Translation2d(1,0), 90),
                "The Robot needs to rotate -90 degrees");
        //From first quadrant to second quadrant
        assertEquals(135, ShootAndRun.getYawToVirtualGoal(new Translation2d(-1,1), 0),
                "The Robot needs to rotate 135 degrees");
        assertEquals(180, ShootAndRun.getYawToVirtualGoal(new Translation2d(-1,0), 0),
                "The Robot needs to rotate 180 degrees");
        //From first quadrant to third quadrant
        assertEquals(-135, ShootAndRun.getYawToVirtualGoal(new Translation2d(-1,-1), 0),
                "The Robot needs to rotate -135 degrees");
        //From first quadrant to fourth quadrant
        assertEquals(-90, ShootAndRun.getYawToVirtualGoal(new Translation2d(0,-1), 0),
                "The Robot needs to rotate -90 degrees");
        assertEquals(-45, ShootAndRun.getYawToVirtualGoal(new Translation2d(1,-1), 0),
                "The Robot needs to rotate -45 degrees");

    }
    @Test
    public void testGetShootingDistance(){
        assertEquals(Math.sqrt(2), ShootAndRun.getShootingDistance(new Translation2d(1,1)),
                "The target is 1.41 meters away from the robot");
        assertEquals(Math.sqrt(2), ShootAndRun.getShootingDistance(new Translation2d(-1,1)),
                "The target is 1.41 meters away from the robot");
        assertEquals(Math.sqrt(2), ShootAndRun.getShootingDistance(new Translation2d(-1,-1)),
                "The target is 1.41 meters away from the robot");
        assertEquals(Math.sqrt(2), ShootAndRun.getShootingDistance(new Translation2d(1,-1)),
                "The target is 1.41 meters away from the robot");




    }

    @Test
    public void everything() {
//        double distance = 3.5;
//        double yaw = 0;
//        Translation2d goal = ShootAndRun.calculateCurrentGoal(distance, yaw);
//        Translation2d virtual = ShootAndRun.calculateVirtualGoal(goal, new ChassisSpeeds(0, 1, 0), Utils.timeByDistance(distance));
//        double offset = ShootAndRun.getYawToVirtualGoal(virtual, 0);
//        double distance_offset = ShootAndRun.getShootingDistance(virtual);
//        System.out.println(offset);
//        System.out.println(distance_offset);
    }

}
