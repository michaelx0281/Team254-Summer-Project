package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.elevator.Elevator;

public class ElevatorTest {

  Elevator elevator = Elevator.create();

  @BeforeEach
  public void setup() {
    setupHAL();
  }

  @Test
  public void movesToGoal() {
    double goal = 2;
    elevator.moveToHeight(Meters.of(2));
    assertEquals(goal, elevator.retrieveHeight());
  }
}
