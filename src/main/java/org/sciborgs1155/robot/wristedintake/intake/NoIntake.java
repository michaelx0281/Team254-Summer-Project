package org.sciborgs1155.robot.wristedintake.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/**
 * NoIntake
 */
public class NoIntake implements IntakeIO{

    @Override
    public void setVoltage(double voltage) {}

    @Override
    public Measure<Velocity<Angle>> getSpeed() {
        return RadiansPerSecond.of(0);
    }
}