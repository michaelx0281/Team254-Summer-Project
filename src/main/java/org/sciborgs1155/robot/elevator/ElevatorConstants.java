package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import monologue.Annotations.Log;

public final class ElevatorConstants {
  public static final double kP = 10.137;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  @Log.NT public static final double kV = 11.191;
  @Log.NT public static final double kA = 1.3054;
  @Log.NT public static final double kS = 0;
  @Log.NT public static final double kG = 0.14014;

  public static final Measure<Velocity<Distance>> maxVelocity = MetersPerSecond.of(6);

  @Log.NT
  public static final Measure<Velocity<Velocity<Distance>>> maxAccel =
      MetersPerSecondPerSecond.of(5);

  // public static final TrapezoidProfile profile = new TrapezoidProfile(constraints);

  public static final double rotationFactor = 1.0;

  // Values needed for Sim
  public static final double massKg = 4; // 3
  public static final double radius = Units.inchesToMeters(2);
  public static final double gearing = 30; //TODO OML I DIDN'T CHANGE IT TO THE RIGHT GEARING VALUES -> RERUN SYSID LATER

  public static final double DISTANCE_PER_PULSE = 1.0;
  public static final double ROTOR_OFFSET = 0.0;
  public static final Measure<Distance> CONVERSION = Meters.of(Units.inchesToMeters(6));
}
