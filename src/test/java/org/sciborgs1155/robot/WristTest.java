package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.wristedintake.wrist.Wrist;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.TestingUtil.fastForward;
import static org.sciborgs1155.lib.TestingUtil.run;
import static org.sciborgs1155.lib.TestingUtil.setupHAL;

public class WristTest {
    Wrist wrist;
    double DELTA = 3E-8;

    @BeforeEach
    public void setup() {
        setupHAL();
        wrist = Wrist.create();
    }

    @Test
    public void moveToDesiredAngleRads() {
        
        Measure<Angle> goalAngleRadians = Radians.of(Units.degreesToRadians(42));
        run(wrist.setDesiredAngle(Radians.of(Units.degreesToRadians(42))));
        fastForward(2000);
        assertEquals(goalAngleRadians, wrist.goalAngleRadians());
        // assertTrue(() -> goalAngleRadians == wrist.goalAngleRadians());
        assertEquals(goalAngleRadians.in(Radians), wrist.getAngle().in(Radians), DELTA);
    }
}
