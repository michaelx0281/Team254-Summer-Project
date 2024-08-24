package org.sciborgs1155.robot.wristedintake.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * SimIntake
 */
public class SimIntake implements IntakeIO{

    DCMotorSim sim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(0, 0),
            DCMotor.getMiniCIM(2), //will change this value when I get the internet back again and check the tech-binder
            0
        );

    @Override
    public void setVoltage(double voltage) {
        sim.setInput(voltage);
        sim.update(0.02);
    }

    @Override
    public Measure<Velocity<Angle>> getSpeed() {
        return RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
    }


    
}