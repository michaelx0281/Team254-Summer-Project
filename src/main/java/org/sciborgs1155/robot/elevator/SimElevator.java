package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class SimElevator implements ElevatorIO {
  ElevatorSim sim =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(DCMotor.getMiniCIM(4), massKg, radius, gearing),
          DCMotor.getMiniCIM(4),
          2, // adjust these values later to the right ones
          6,
          true,
          4);
  TalonFX talon = new TalonFX(0);
  Encoder encoder = new Encoder(0, 1);

  // CANcoder encoder = talon.canCod;
  // SimDeviceSim motor = new SimDeviceSim(0);
  // Matrix<N2,N1> currentState;
  // Matrix<N1,N1> setpoint;

  public SimElevator() {
    encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
  }

  EncoderSim simEncoder = new EncoderSim(encoder);

  @Override
  public void moveToSetpoint(Measure<Velocity<Distance>> setpoint) {
    // sim.setInput(setpoint.in(MetersPerSecond));
    // sim.update(0.02);
    // sim.setState(sim.getPositionMeters(), sim.getVelocityMetersPerSecond());
    // currentState.set(1, 1, sim.getPositionMeters());
    // currentState.set(1, 2, sim.getVelocityMetersPerSecond());
    // this.setpoint.fill(setpoint.in(MetersPerSecond));

    // * RobotController.getBatteryVoltage()
    if(sim.hasHitLowerLimit() || sim.hasHitUpperLimit()){
      System.out.println("massive error");
    }

    /*sets */
    TalonFXSimState talonSim = talon.getSimState();
    talonSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var motorVoltage = talonSim.getMotorVoltage();

    sim.setInputVoltage(motorVoltage);

    sim.update(0.02);
    System.out.println("setpoint: " + setpoint.in(MetersPerSecond));

    talon.setVoltage(setpoint.in(MetersPerSecond));
    System.out.println("TalonSetSpeed: " + talon.get());
    System.out.println("TalonVELO: " + talon.getVelocity());
    System.out.println("SimTalonVoltage: " + talonSim.getMotorVoltage());

    System.out.println("EncoderVal: " + encoder.getDistance());
    System.out.println("SimEncoderVal: " + simEncoder.getDistance());
    System.out.println("SimElevatorOutput: " + sim.getOutput());

    System.out.print("height: " + simEncoder.getDistance() + " ");

    // RoboRioSim.setVInVoltage(
    //   BatterySim.calculateDefaultBatteryLoadedVoltage(
    //     sim.getCurrentDrawAmps()));
    // System.out.println("");

    // sim.setState(sim.getPositionMeters(), sim.getVelocityMetersPerSecond());

    // sim.updateX(
    //   currentState,
    //   this.setpoint,
    //   0.02
    // );
    simEncoder.setDistance(sim.getPositionMeters());
  }

  @Override
  public double heightFromBase() {
    return simEncoder.getDistance();
  }
}
