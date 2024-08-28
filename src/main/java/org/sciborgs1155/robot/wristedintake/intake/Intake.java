// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.wristedintake.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.wristedintake.intake.IntakeConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  /** Creates a new Intake. */
  public Intake(IntakeIO hardware) {
    this.hardware = hardware;
  }

  public static Intake create() {
    return Robot.isReal() ? new Intake(new RealIntake()) : new Intake(new SimIntake());
  }

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
