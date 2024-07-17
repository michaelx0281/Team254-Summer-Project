package org.sciborgs1155.robot.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public interface ElevatorIO {
  public void moveToSetpoint(Measure<Velocity<Distance>> setpoint);

  public double heightFromBase();
}
