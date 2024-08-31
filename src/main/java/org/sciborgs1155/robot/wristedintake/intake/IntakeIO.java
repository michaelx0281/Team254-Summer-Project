package org.sciborgs1155.robot.wristedintake.intake;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/** IntakeIO */
public interface IntakeIO {
  public void setVoltage(double voltage);

  public Measure<Velocity<Angle>> getSpeed();

  public Measure<Velocity<Velocity<Angle>>> getAccel(); //TODO might get rid of this some other time, seems kinda a bad idea to be using this

  public Measure<Angle> getPositionRads(); // it makes almost no sense for this method to exist for a flyway, however the log consumer seems to want it so here it is

  public Measure<Voltage> voltage();
}
