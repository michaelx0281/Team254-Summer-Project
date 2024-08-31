package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.mechanisms.DifferentialMechanism.DisabledReason;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimElevator implements ElevatorIO {

  private Measure<Voltage> volts;

  ElevatorSim sim =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(DCMotor.getMiniCIM(4), massKg, radius, gearing),
          DCMotor.getMiniCIM(4),
          -20, // adjust these values later to the right ones
          20,
          true,
          0);

  @Override
  public void setVoltage(Measure<Voltage> volts) {
    this.volts = volts;
    sim.setInputVoltage(volts.in(Volts));
    sim.update(0.02);
  }

  @Override
  public double heightFromBase() {
    // System.out.println("Height: " + sim.getPositionMeters());
    return sim.getPositionMeters();
  }

  @Override
  public Measure<Velocity<Distance>> getVelocity() {
    return MetersPerSecond.of(sim.getVelocityMetersPerSecond());
  }

  @Override
  public Measure<Voltage> voltage() {
    return volts;
  }
}
