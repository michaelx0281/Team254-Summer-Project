package org.sciborgs1155.robot.wristedintake.wrist;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * SimWrist
 */
public class SimWrist implements WristIO{

    SingleJointedArmSim sim = 
    new SingleJointedArmSim(
        DCMotor.getMiniCIM(1),
        0,
        0,
        0,
        0,
        0,
        true,
        0);
        
    @Override
    public void setVoltage(double volts) {
        sim.setInput(volts);
        sim.update(volts);
    }

    @Override
    public Measure<Angle> getPositionRadians() {
        return Radians.of(sim.getAngleRads());
    }

    @Override
    public Measure<Velocity<Angle>> getSpeed() {
        return RadiansPerSecond.of(sim.getVelocityRadPerSec());
    }

    
}