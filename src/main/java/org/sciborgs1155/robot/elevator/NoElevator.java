package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public class NoElevator implements ElevatorIO {

  @Override
  public void setVoltage(Measure<Voltage> volts) {}

  @Override
  public double heightFromBase() {
    return 0;
  }

  @Override
  public Measure<Velocity<Angle>> getSpeed() {
    return RadiansPerSecond.of(0);
  }

  @Override
  public Measure<Voltage> voltage() {
    return Volts.of(0);
  }
}
