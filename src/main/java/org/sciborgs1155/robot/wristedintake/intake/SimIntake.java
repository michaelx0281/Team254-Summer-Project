package org.sciborgs1155.robot.wristedintake.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.wristedintake.intake.IntakeConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** SimIntake */
public class SimIntake implements IntakeIO {

  private Measure<Voltage> volts;
  private double initVelo = 0;

  FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getMiniCIM(2),
              MOI,
              GEARING), 
          // the tech-binder
          DCMotor.getMiniCIM(2),
          GEARING);

  @Override
  public void setVoltage(double voltage) { //TODO use units for this import
    this.volts = Volts.of(voltage);
    sim.setInputVoltage(voltage);
    sim.update(0.02);
  }

  @Override
  public Measure<Velocity<Angle>> getSpeed() {
    return RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
  }

  @Override
  public Measure<Voltage> voltage() {
    return volts;
  }

  @Override
  public Measure<Velocity<Velocity<Angle>>> getAccel() {
    double nextVelo = sim.getAngularVelocityRadPerSec();
    double accel = (nextVelo - initVelo) / 0.02;
    initVelo = nextVelo;
    return RadiansPerSecond.per(Second).of(accel);
  }
}
