// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.pneumatics.hanger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.pneumatics.PneumaticsIO;

public class Hanger extends SubsystemBase {
  private PneumaticsIO hardware;

  public Hanger(PneumaticsIO hardware) {
    this.hardware = hardware;
  }

  /* Creates a new hanger */
  public static Hanger create() {
    return Robot.isReal() ? new Hanger(new RealHanger()) : new Hanger(new SimHanger());
  }

  public static Hanger none() {
    return new Hanger(new NoHanger());
  }

  public Command extend() {
    return runOnce(() -> hardware.extend());
  }

  public Command retract() {
    return runOnce(() -> hardware.retract());
  }

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
}
