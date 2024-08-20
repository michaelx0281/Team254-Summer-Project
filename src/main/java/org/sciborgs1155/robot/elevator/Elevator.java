package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import org.sciborgs1155.lib.TestingUtil;

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
  @Log.NT private double goalHeight = 2;

  public Elevator(ElevatorIO elevator) {
    this.elevator = elevator;
    elevatorVisual.setColor(new Color8Bit(Color.kAqua));
    SmartDashboard.putData("elevator2D", mech);
  }

  

  public static Elevator create() {
    return Robot.isReal() ? new Elevator(new RealElevator()) : new Elevator(new SimElevator());
  }

  public void setGoal(double heightInMeters) {
    this.goalHeight = heightInMeters;
    pid.setGoal(goalHeight);
  }

  public Command setGoal(Measure<Distance> height){
    return runOnce(() -> setGoal(height.in(Meters)));
  }

  public Command moveToHeight() {
    return run(
        () -> {
          System.out.println("Goal Position: "+ pid.getGoal().position);
          double pidOutput = pid.calculate(elevator.heightFromBase());
          double ffOutput = ff.calculate(pid.getSetpoint().position);
          System.out.println("output: " + (pidOutput + ffOutput) + " pidOutput: " + pidOutput);
          elevator.setVoltage(Volts.of(pidOutput + ffOutput));
          elevatorVisual.setLength(elevator.heightFromBase());
          position = elevator.heightFromBase(); 
        });
  }

  public double retrieveHeight() {
    return elevator.heightFromBase();
  }
}
