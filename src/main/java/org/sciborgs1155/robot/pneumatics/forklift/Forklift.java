// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.pneumatics.forklift;

import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.pneumatics.PneumaticsIO;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Forklift extends SubsystemBase {
  PneumaticsIO hardware;
  /** Creates a new Forklift. */
  public Forklift(PneumaticsIO hardware) {
    this.hardware = hardware;
  }

  public static Forklift create(){
    return Robot.isReal() ? new Forklift(new RealForklift()) : new Forklift(new SimForklift());
  }

  public static Forklift none(){
    return new Forklift(new NoForklift());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
