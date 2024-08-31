// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.wristedintake.wrist;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.wristedintake.wrist.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Wrist extends SubsystemBase implements Logged {
  WristIO hardware;

  @Log.NT
  private ProfiledPIDController pid =
      new ProfiledPIDController(kP, kI, kD, new Constraints(MAX_VELO, MAX_ACCEL));

  @Log.NT private ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);
  @Log.NT private double desiredAngle = Radians.of(0).in(Radians);
  @Log.NT private double positionRadians = Radians.of(0).in(Radians);
  
  private SysIdRoutine routine;

  public Wrist(WristIO hardware) {
    this.hardware = hardware;

    routine = 
      new SysIdRoutine(
        new SysIdRoutine.Config(), //the line below possibly needs a form of log to be added
        new SysIdRoutine.Mechanism(volts -> hardware.setVoltage(volts.in(Volts)), null, this)); //TODO change and use unit library units for output of this method
    
        SmartDashboard.putData("wrist dynamic forward", wristSysidDynamic(Direction.kForward));
        SmartDashboard.putData("wrist dynamic backward", wristSysidDynamic(Direction.kReverse));
        SmartDashboard.putData("wrist quasistatic forward", wristSysidDynamic(Direction.kForward));
        SmartDashboard.putData("wrist quasisttic backward", wristSysidDynamic(Direction.kReverse));
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
    return runOnce(() -> setGoalAngle(angle.in(Radians)));
  }

  public Command stopImmediately() {
    return runOnce(() -> hardware.setVoltage(0));
  }

  public Measure<Angle> goalAngleRadians() {
    return Radians.of(desiredAngle);
  }

   /*NEW FORMATTING - chain two private comands together here! */
  /* setDesiredAngle 2.0 */
  public Command setDesiredAngle(Measure<Angle> angle) {
    return setGoalAngle(angle).andThen(moveToGoalAngle());
  }

  public Measure<Angle> getAngle() {
    return hardware.getPositionRadians();
  }

  public Command wristSysidDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public Command wristSysidQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // pid.setGoal(desiredAngle);
  }
}