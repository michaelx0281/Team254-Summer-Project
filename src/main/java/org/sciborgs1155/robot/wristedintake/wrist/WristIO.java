package org.sciborgs1155.robot.wristedintake.wrist;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/**
 * WristIO
 */

public interface WristIO {
    public void setVoltage(double volts);
    public Measure<Angle> getPositionRadians();
    public Measure<Velocity<Angle>> getSpeed();
}