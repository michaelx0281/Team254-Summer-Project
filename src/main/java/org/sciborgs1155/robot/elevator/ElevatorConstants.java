package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public final class ElevatorConstants {
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double kS = 0.0;
  public static final double kG = 1.0;

  public static final Measure<Velocity<Distance>> maxVelocity = MetersPerSecond.of(1.0);
  public static final Measure<Velocity<Velocity<Distance>>> maxAccel =
      MetersPerSecondPerSecond.of(1.0);

  public static final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(maxVelocity, maxAccel);

  // public static final TrapezoidProfile profile = new TrapezoidProfile(constraints);

  public static final double rotationFactor = 1.0;

  // Values needed for Sim
  public static final double massKg = 10.0;
  public static final double radius = 0.05;
  public static final double gearing = 1;
}
