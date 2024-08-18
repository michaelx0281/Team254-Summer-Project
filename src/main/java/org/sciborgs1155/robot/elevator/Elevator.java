package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Elevator extends SubsystemBase implements Logged {

  private ElevatorIO elevator;
  private Mechanism2d mech = new Mechanism2d(7, 7);
  private MechanismRoot2d root = mech.getRoot("elevator", 4, 0);
  private MechanismLigament2d elevatorVisual =
      root.append(new MechanismLigament2d("elevator cart", 3, 90));

  @Log.NT private ProfiledPIDController pid = new ProfiledPIDController(kP, kP, kP, constraints);
  @Log.NT private ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

  @Log.NT private double position = 0;

  public Elevator(ElevatorIO elevator) {
    this.elevator = elevator;
    elevatorVisual.setColor(new Color8Bit(Color.kAqua));
    SmartDashboard.putData("elevator2D", mech);
  }

  public static Elevator create() {
    return Robot.isReal() ? new Elevator(new RealElevator()) : new Elevator(new SimElevator());
  }

  public Command moveToHeight(Measure<Distance> height) {
    pid.setGoal(height.in(Meters));
    for(int i =0; i<1; i++){
      System.out.println("Goal: " + height.in(Meters));
      System.out.println("Just a confirmation that printing works outside of the close-control loop");
    }

    return run(
        () -> {
          double pidOutput = pid.calculate(elevator.heightFromBase());
          double ffOutput = ff.calculate(pid.getSetpoint().velocity);
          System.out.println("output: " + (pidOutput + ffOutput) + " pidOutput: " + pidOutput);
          elevator.moveToSetpoint(MetersPerSecond.of(pidOutput + ffOutput));
          elevatorVisual.setLength(elevator.heightFromBase());
          position = elevator.heightFromBase();
        });
  }

  public Command maintainPosition() {
    return moveToHeight(Meters.of(elevator.heightFromBase()));
  }

  public double retrieveHeight() {
    return elevator.heightFromBase();
  }
}
