package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimElevator implements ElevatorIO{
    public SimElevator(){}

    DCMotor gearbox = 
        new DCMotor( // adjust values to be the right ones and add constants later
            1,
             1,
              1,
               1,
                1,
                 4);

    ElevatorSim sim = 
        new ElevatorSim
            (LinearSystemId.createElevatorSystem(
                gearbox,
                massKg,
                radius,
                gearing
            ),
             gearbox, 
              3, //adjust these values later to the right ones
               6,
                true,
                 3);

    @Override
    public void moveToSetpoint(Measure<Velocity<Distance>> setpoint) {
        sim.setInput(setpoint.in(MetersPerSecond), setpoint.in(MetersPerSecond), setpoint.in(MetersPerSecond), setpoint.in(MetersPerSecond));
    }

    @Override
    public double heightFromBase() {
        return sim.getPositionMeters();
    }

  
}
