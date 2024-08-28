package org.sciborgs1155.robot.wristedintake.intake;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/** IntakeIO */
public interface IntakeIO {
  public void setVoltage(double voltage);

  public Measure<Velocity<Angle>> getSpeed();
}
