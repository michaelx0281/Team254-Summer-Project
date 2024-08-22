package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import monologue.Annotations.Log;

public final class ElevatorConstants {
  public static final double kP = 0.3;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  @Log.NT
  public static final double kV = 1.7;
  @Log.NT
  public static final double kA = 0;
  @Log.NT
  public static final double kS = 1;
  @Log.NT
  public static final double kG = 5.9506;

  public static final Measure<Velocity<Distance>> maxVelocity = 
  MetersPerSecond.of(0.4);
  public static final Measure<Velocity<Velocity<Distance>>> maxAccel =
      MetersPerSecondPerSecond.of(10);

  // public static final TrapezoidProfile profile = new TrapezoidProfile(constraints);

  public static final double rotationFactor = 1.0;

  // Values needed for Sim
  public static final double massKg = 10;
  public static final double radius = 1;
  public static final double gearing = 30;

  public static final double DISTANCE_PER_PULSE = 1.0;
  public static final double ROTOR_OFFSET = 0.0;
}
