// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.wristedintake.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.wristedintake.intake.IntakeConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Intake extends SubsystemBase implements Logged {
  private IntakeIO hardware;
  // @Log.NT private Measure<Velocity<Angle>> velocityRadsPS = RadiansPerSecond.of(0);
  @Log.NT private double velocityRadsPS = RadiansPerSecond.of(0).in(RadiansPerSecond);

  // TODO make constants for fb and ff controllers
  @Log.NT private PIDController pid = new PIDController(kP, kI, kD);
  @Log.NT private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

  private SysIdRoutine routine;


  /* Creates a new Intake. */
  public Intake(IntakeIO hardware) {
    this.hardware = hardware;

    routine = 
      new SysIdRoutine(
        new SysIdRoutine.Config(), //the line below possibly needs a form of log to be added
        new SysIdRoutine.Mechanism(volts -> hardware.setVoltage(
          volts.in(Volts)),
           log -> {
            log.motor("intake")
              .voltage(hardware.voltage())
              .angularVelocity(hardware.getSpeed())
              .angularPosition(hardware.getPositionRads());
           },
            this)); //TODO change and use unit library units for output of this method
    
        SmartDashboard.putData("intake dynamic forward", intakeSysidDynamic(Direction.kForward));
        SmartDashboard.putData("intake dynamic backward", intakeSysidDynamic(Direction.kReverse));
        SmartDashboard.putData("intake quasistatic forward", intakeSysidDynamic(Direction.kForward));
        SmartDashboard.putData("intake quasistatic backward", intakeSysidDynamic(Direction.kReverse));

  }


  /* Creates a new Intake subsystem */
  public static Intake create() {
    return Robot.isReal() ? new Intake(new RealIntake()) : new Intake(new SimIntake());
  }

  /* Creates an empty Intake subsystem */
  public static Intake none() {
    return new Intake(new NoIntake());
  }

  /* Runs a command to get intake to ramp up smoothly to a desired speed.*/
  public Command setDesiredSpeed(double speed) {
    return run(
        () -> {
          double pidOutput = pid.calculate(hardware.getSpeed().in(RadiansPerSecond), speed);
          double ffOutput =
              ff.calculate(hardware.getSpeed().in(RadiansPerSecond), pid.getSetpoint());

          hardware.setVoltage(pidOutput + ffOutput);
          velocityRadsPS = hardware.getSpeed().in(RadiansPerSecond);
        });
  }

  /* The method of choice for an intake - excuse me - BECAUSE ITS A GODD*MN INTAKE AND DOESN"T NEED A PID (Totally not bc my pid isn't working very well...) */
  public Command directSetVoltage(Measure<Voltage> volts) {
    return run(() -> hardware.setVoltage(volts.in(Volts)));
  }

  /* Returns velocity of intake in RadsPS */
  public Measure<Velocity<Angle>> getSpeed() {
    return hardware.getSpeed();
  }

  /* Outakes object at given speed */
  public Command outake(double speed) {
    return setDesiredSpeed(-speed);
  }

  /* Stops intake gradually. */
  public Command stop() {
    return setDesiredSpeed(0);
  }

  /* Stops intake. */
  public Command stopImmediately() {
    return runOnce(() -> hardware.setVoltage(0));
  }

  public Command intakeSysidDynamic(SysIdRoutine.Direction direction){
    return routine.dynamic(direction);
  }

  public Command intakeSysidQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
