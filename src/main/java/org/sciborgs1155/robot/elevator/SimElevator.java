package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class SimElevator implements ElevatorIO {
  ElevatorSim sim =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(DCMotor.getMiniCIM(4), massKg, radius, gearing),
          DCMotor.getMiniCIM(4),
          -20, // adjust these values later to the right ones
          20,
          true,
          0);

  @Override
  public void setVoltage(Measure<Voltage> volts)   {
    sim.setInput(volts.in(Volts)); 
    sim.update(0.02);
  }

  @Override
  public double heightFromBase() {
    // System.out.println("Height: " + sim.getPositionMeters());
    return sim.getPositionMeters();
  }

  @Override
  public Measure<Velocity<Angle>> getSpeed() {
    return RadiansPerSecond.of(sim.getVelocityMetersPerSecond() / 1 * 2*Math.PI); //TODO change '1' to a constant that scales Rotations to Meters
  }
}
