package org.sciborgs1155.robot.wristedintake.intake;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/** NoIntake */
public class NoIntake implements IntakeIO {

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public Measure<Velocity<Angle>> getSpeed() {
    return RadiansPerSecond.of(0);
  }

  @Override
  public Measure<Voltage> voltage(){
    return Volts.of(0);
  }

  @Override
  public Measure<Velocity<Velocity<Angle>>> getAccel() {
    return RadiansPerSecond.per(Second).of(0);
  }
}
