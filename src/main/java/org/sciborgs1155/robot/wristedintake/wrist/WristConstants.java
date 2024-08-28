package org.sciborgs1155.robot.wristedintake.wrist;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/** WristConstants */
public class WristConstants {

  // TODO give units values to each of the constants.

  public static final double ROTOR_OFFSET = 0;
  public static final double ROTOR_TO_SENSOR_RATIO = 1;
  public static final double MOI = 2.5;
  public static final double GEARING = 150.0; // 150:1 REDUCTION
  public static final Measure<Distance> wristLength = Meters.of(Units.inchesToMeters(6.5));

  public static final double kP = 1.0;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kS = 2;
  public static final double kG = 0.552;
  public static final double kV = 5;
  public static final double kA = 5;

  public static final Measure<Velocity<Distance>> MAX_VELO = MetersPerSecond.of(5);
  public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCEL =
      MetersPerSecondPerSecond.of(5);
}
