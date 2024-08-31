package org.sciborgs1155.robot.wristedintake.wrist;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.wristedintake.wrist.WristConstants.GEARING;
import static org.sciborgs1155.robot.wristedintake.wrist.WristConstants.MOI;
import static org.sciborgs1155.robot.wristedintake.wrist.WristConstants.wristLength;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** SimWrist */
public class SimWrist implements WristIO {

  private Measure<Voltage> volts;

  SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getMiniCIM(1), GEARING, SingleJointedArmSim.estimateMOI(Units.inchesToMeters(5.5), 1), wristLength.in(Meters), 0, Math.PI * 2, true, 0);

  @Override
  public void setVoltage(double volts) {
    this.volts = Volts.of(volts);
    sim.setInputVoltage(volts);
    sim.update(0.02);
    // System.out.println(sim.getVelocityRadPerSec());
  }

  @Override
  public Measure<Angle> getPositionRadians() {
    return Radians.of(sim.getAngleRads());
  }

  @Override
  public Measure<Velocity<Angle>> getSpeed() {
    return RadiansPerSecond.of(sim.getVelocityRadPerSec());
  }

  @Override
  public Measure<Voltage> voltage() {
    return volts;
  }
}
