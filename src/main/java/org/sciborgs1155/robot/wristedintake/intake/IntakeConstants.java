package org.sciborgs1155.robot.wristedintake.intake;

/** IntakeConstants */
public class IntakeConstants {

  // TODO - idk if I should use units api for this?
  public static final double ROTOR_TO_SENSOR_RATIO = 1;
  public static final double ROTOR_OFFSET = 0;
  public static final double GEARING = 5; // I think this is what 5:1 reduction means?
  public static final double MOI = 5;

  public static final double kP = 2;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kV = 0.046; // .306 kV .262 kA for GEARING = 30
  public static final double kA = 0.045; // .046 kV .045 kA for GEARING = 1?
  public static final double kS = 0;
}
