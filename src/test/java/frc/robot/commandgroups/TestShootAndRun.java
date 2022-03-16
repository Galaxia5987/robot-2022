package frc.robot.commandgroups;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;

import org.junit.Test;

public class TestShootAndRun {



    @Test
    public void testCalculateCurrentGoal() {
        assertEquals(new Translation2d(1, 0), ShootAndRun.calculateCurrentGoal(1, 0),
                "Should give coordinates of (1, 0)");
    }
}
