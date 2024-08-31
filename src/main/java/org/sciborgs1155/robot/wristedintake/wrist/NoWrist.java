package org.sciborgs1155.robot.wristedintake.wrist;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.lang.invoke.VolatileCallSite;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/** NoWrist */
public class NoWrist implements WristIO {

  @Override
  public void setVoltage(double volts) {}

  @Override
  public Measure<Angle> getPositionRadians() {
    return Radians.of(0);
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
