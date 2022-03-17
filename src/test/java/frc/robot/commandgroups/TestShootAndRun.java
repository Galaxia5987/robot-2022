package frc.robot.commandgroups;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.Test;
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
        assertEquals(45, ShootAndRun.getYawToVirtualGoal(new Translation2d(1,1)), "should give an output of 45 degrees");
    }
}
