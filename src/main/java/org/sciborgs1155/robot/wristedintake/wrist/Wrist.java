// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.wristedintake.wrist;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.wristedintake.wrist.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Wrist extends SubsystemBase implements Logged {
  WristIO hardware;

  @Log.NT
  ProfiledPIDController pid =
      new ProfiledPIDController(kP, kI, kD, new Constraints(MAX_VELO, MAX_ACCEL));

  @Log.NT ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);
  @Log.NT double desiredAngle = Radians.of(0).in(Radians);
  @Log.NT double positionRadians = Radians.of(0).in(Radians);

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

  private Command moveToGoalAngle() {
    return run(
        () -> {
          // pid.setGoal(desiredAngle);
          double pidOutput = pid.calculate(hardware.getPositionRadians().in(Radians));
          double ffOutput = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);

          System.out.println(
              "PID: "
                  + pidOutput
                  + " FF: "
                  + ffOutput
                  + " Setpoint pos: "
                  + pid.getSetpoint().position
                  + " Setpoint velo: "
                  + pid.getSetpoint().velocity);

          // hardware.setVoltage(pidOutput + ffOutput);
          hardware.setVoltage(pidOutput + ffOutput);
          positionRadians = hardware.getPositionRadians().in(Radians);
        });
  }

  private void setGoalAngle(double angle) {
    /* Clamps the possible angle value to be in between 0 and PI/2 Radians = [0, 90] degrees. */
    desiredAngle =
        MathUtil.clamp(
            angle,
            0,
            Math.PI
                / 2); // TODO get rid of this, this is pretty useless because bounds were already
    // elaborated on previously
  }

  private Command setGoalAngle(Measure<Angle> angle) {
    System.out.println("Angle set");
    return runOnce(() -> setGoalAngle(angle.in(Radians)));
  }

  public Command stopImmediately() {
    return runOnce(() -> hardware.setVoltage(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // pid.setGoal(desiredAngle);
  }

  /*NEW FORMATTING - chain two private comands together here! */
  /* setDesiredAngle 2.0 */

  public Command setDesiredAngle(Measure<Angle> angle) {
    return setGoalAngle(angle).andThen(moveToGoalAngle());
  }
}
