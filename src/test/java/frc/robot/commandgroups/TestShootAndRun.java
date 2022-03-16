package frc.robot.commandgroups;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;

import org.junit.Test;

public class TestShootAndRun {


    @Test
    public void testCalculateCurrentGoal() {
        //First quadrant
        assertEquals(new Translation2d(1, 0), ShootAndRun.calculateCurrentGoal(1, 0),
                "Should give coordinates of (1, 0)");
        assertEquals(new Translation2d(0,1), ShootAndRun.calculateCurrentGoal(1, 90),
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
}
