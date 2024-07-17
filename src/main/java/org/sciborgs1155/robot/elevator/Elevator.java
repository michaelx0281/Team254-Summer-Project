package org.sciborgs1155.robot.elevator;

import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import org.sciborgs1155.robot.Robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    ElevatorIO elevator;

    public Elevator(ElevatorIO elevator){
        this.elevator = elevator;
    }

    public static Elevator create(){
        return Robot.isReal() ? new Elevator(new RealElevator()) : new Elevator(new SimElevator());
    }

    public Command moveToHeight(Measure<Distance> height){
        return Commands.run(() -> {
            ProfiledPIDController pid = new ProfiledPIDController(kP, kP, kP, constraints);
            ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV);

            double pidOutput = pid.calculate(elevator.heightFromBase());
            double ffOutput = ff.calculate(pid.getSetpoint().velocity);

            elevator.moveToSetpoint(pidOutput + ffOutput);
        });
    }


}
