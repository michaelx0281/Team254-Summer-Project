package org.sciborgs1155.robot.elevator;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public interface ElevatorIO {
  public void setVoltage(Measure<Voltage> volts);

  public Measure<Velocity<Distance>> getVelocity();

  public double heightFromBase();

  public Measure<Voltage> voltage();
}
