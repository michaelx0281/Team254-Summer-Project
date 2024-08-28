package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.fastForward;
import static org.sciborgs1155.lib.TestingUtil.run;
import static org.sciborgs1155.lib.TestingUtil.setupHAL;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.wristedintake.intake.Intake;

public class IntakeTest {
  Intake intake = Intake.create();

  @BeforeEach
  public void setup() {
    setupHAL();
  }

  @Test
  public void movesToVelocitySetpoint() {
    double TOLERANCE = 8E-3;
    double velocitySetpoint = 4;
    run(intake.setDesiredSpeed(4));
    fastForward(2000);
    assertEquals(velocitySetpoint, intake.getSpeed().in(RadiansPerSecond), TOLERANCE);
  }
}
