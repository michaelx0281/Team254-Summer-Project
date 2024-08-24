package org.sciborgs1155.robot.wristedintake.wrist;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/**
 * NoWrist
 */
public class NoWrist implements WristIO{

    @Override
    public void setVoltage(double volts) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    public Measure<Angle> getPositionRadians() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPositionRadians'");
    }

    @Override
    public Measure<Velocity<Angle>> getSpeed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSpeed'");
    }

    
}