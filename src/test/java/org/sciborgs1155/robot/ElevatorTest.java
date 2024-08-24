package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.elevator.Elevator;

public class ElevatorTest {

  Elevator elevator = Elevator.create();

  // @BeforeEach
  // public void setup() {
  //   setupHAL();
  // }

  // @Test
  // public void movesToGoal() {
  //   double goal = 2;
  //   elevator.setGoal(Meters.of(2));
  //   System.out.println(elevator.goal());
  //   elevator.moveToHeight();
  //   fastForward(5000);
  //   assertEquals(goal, elevator.retrieveHeight());
  // }

  // // @AfterEach
  // public void close(){
  //   elevator.close();
  // }
}
