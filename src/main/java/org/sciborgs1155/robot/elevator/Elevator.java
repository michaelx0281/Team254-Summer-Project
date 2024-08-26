package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.drive.DriveConstants.CONSTRAINTS;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import java.io.InputStream;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.sciborgs1155.lib.TestingUtil;
import org.sciborgs1155.lib.Tuning;

import static org.sciborgs1155.lib.Tuning.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Elevator extends SubsystemBase implements Logged {

  private ElevatorIO hardware;
  private Mechanism2d mech = new Mechanism2d(7, 7);
  private MechanismRoot2d root = mech.getRoot("elevator", 4, 0);
  private MechanismLigament2d elevatorVisual =
      root.append(new MechanismLigament2d("elevator cart", 3, 90));

   private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(maxVelocity, maxAccel);  

  @Log.NT private ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kD, constraints, 0.02);
  @Log.NT  private ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

  private SysIdRoutine routine;

  @Log.NT private double position = 0;
  @Log.NT public boolean stop = false;
  @Log.NT public double goalHeight = 0;

  public Elevator(ElevatorIO hardware) {
    this.hardware = hardware;
    elevatorVisual.setColor(new Color8Bit(Color.kAqua));
    SmartDashboard.putData("elevator2D", mech);
    pid.setTolerance(1E-3, 1E-3);

    routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine
        .Mechanism(
          volts -> hardware.setVoltage(volts), 
          null, 
          this));

    SmartDashboard.putData("elevator quasistatic forward", elevatorSysidDynamic(Direction.kForward));
    SmartDashboard.putData("elevator quasistatic backward", elevatorSysidDynamic(Direction.kForward));
    SmartDashboard.putData("elevator quasistatic forward", elevatorSysidQuasistatic(Direction.kForward));
    SmartDashboard.putData("elevator quasistatic backward", elevatorSysidQuasistatic(Direction.kForward));
  }

  /*Creates an elevator */
  public static Elevator create() {
    return Robot.isReal() ? new Elevator(new RealElevator()) : new Elevator(new SimElevator());
  }

  /*Creates an empty elevator subsystem with no hardware */
  public static Elevator none() {
    return new Elevator(new NoElevator());
  }

  private void setGoal(double height) {
    goalHeight = height;
    pid.setGoal(goalHeight);
  }

  public Command setGoal(Measure<Distance> height) {
    return runOnce(() -> setGoal(height.in(Meters)));
  }

  double initVelo = 0;
  double nextVelo;
  public Command moveToHeight() {
    System.out.println("Running elevator command.. ");

    return run(
        () -> {
          nextVelo = hardware.getSpeed().in(RadiansPerSecond);
        
          double pidOutput = pid.calculate(hardware.heightFromBase());
          double ffOutput = ff.calculate(pid.getSetpoint().velocity, (nextVelo - initVelo)); // (nextVelo - initVelo)/0.8

          initVelo = nextVelo;

          System.out.println("Pid setpoint velo and position: " + pid.getSetpoint().velocity+ " and " + pid.getSetpoint().position);
          System.out.println("ffoutput: " + (ffOutput) + " pidOutput: " + pidOutput + " pid velo setpoint: " + pid.getSetpoint().velocity + " pid position setpoint: " + pid.getSetpoint().position
          + " goal p: " + pid.getGoal().position + " goal v" + pid.getGoal().velocity);
          
          hardware.setVoltage(Volts.of(pidOutput+ffOutput));
          position = hardware.heightFromBase(); 
        }).withName("moveing to goal height in meters...");
  }

  public double retrieveHeight() {
    return hardware.heightFromBase();
  }

  public double goal(){
    return goalHeight;
  }

  public Command elevatorSysidDynamic(SysIdRoutine.Direction direction){
    return routine.dynamic(direction);
  }

  public Command elevatorSysidQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  
  @Override
  public void simulationPeriodic() {
      // TODO Auto-generated method stub
      super.simulationPeriodic();
      pid.setGoal(goalHeight);
  }
}
