// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.wristedintake.wrist;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;

public class Wrist extends SubsystemBase {
  WristIO hardware;
  @Log.NT ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
  ArmFeedforward ff = new ArmFeedforward(0, 0, 0);

  Measure<Angle> desiredAngle = Radians.of(0);

  public Wrist(WristIO hardware) {
    this.hardware = hardware;
  }

  /* Creates a new wrist */
  public static Wrist create() {
    return Robot.isReal() ? new Wrist(new RealWrist()) : new Wrist(new SimWrist());
  }

  /* Creates empty wrist subsystem */
  public static Wrist none() {
    return new Wrist(new NoWrist());
  }

  public Command setDesiredAngle(Measure<Angle> angle) {
    /* Clamps the possible angle value to be in between 0 and PI/2 Radians = [0, 90] degrees. */
    desiredAngle = Radians.of(MathUtil.clamp(angle.in(Radians), 0, Math.PI/2));
    return run(() -> {
      double pidOutput = pid.calculate(hardware.getPositionRadians().in(Radians));
      double ffOutput = ff.calculate(pid.getSetpoint().position, hardware.getSpeed().in(RadiansPerSecond));

      hardware.setVoltage(pidOutput + ffOutput);
    });
  }

  public Command stopImmediately () {
    return runOnce(() -> hardware.setVoltage(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pid.setGoal(desiredAngle.in(Radians));
  }
}
