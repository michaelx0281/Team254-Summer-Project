package org.sciborgs1155.robot.wristedintake.wrist;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/**
 * NoWrist
 */
public class NoWrist implements WristIO{

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

    
}