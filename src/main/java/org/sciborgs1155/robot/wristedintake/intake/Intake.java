// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.wristedintake.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;

public class Intake extends SubsystemBase {
  private IntakeIO hardware;

  // TODO make constants for fb and ff controllers
  @Log.NT private PIDController pid = new PIDController(0, 0, 0);
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0, 0);

  /** Creates a new Intake. */
  public Intake(IntakeIO hardware) {
    this.hardware = hardware;
  }

  /* Runs a command to get intake to ramp up smoothly to a desired speed.*/
  public Command setDesiredSpeed(double speed) {
    return run(() -> {
      double pidOutput = pid.calculate(hardware.getSpeed().in(RadiansPerSecond), speed);
      double ffOutput = ff.calculate(hardware.getSpeed().in(RadiansPerSecond), 0);

      hardware.setVoltage(pidOutput + ffOutput);
    });
  }

  /* Outakes object at given speed */
  public Command outake(double speed){
    return setDesiredSpeed(-speed);
  }

  /* Stops intake gradually. */
  public Command stop(){
    return setDesiredSpeed(0);
  }

  /* Stops intake. */
  public Command stopImmediately(){
    return runOnce(() -> hardware.setVoltage(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
