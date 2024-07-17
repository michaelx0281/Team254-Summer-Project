package org.sciborgs1155.robot.elevator;

import com.ctre.phoenix6.controls.jni.ControlJNI;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import java.io.Console; //wtf is this?

public class RealElevator implements ElevatorIO{

    public RealElevator() {
        lead.setPosition(0);
    }

// The talons (there were 4 implemented in the CheesyPuffs' code)
    private TalonFX 
        lead, 
        rFollower, 
        lFollowerA, 
        lFollowerB;

    private Measure<Distance> height = Meters.of(0.0);

    public RealElevator(TalonFX lead, TalonFX rFollower, TalonFX lFollowerA, TalonFX lFollowerB){
        this.lead = lead;
        this.rFollower = rFollower;
        this.lFollowerA = lFollowerA;
        this.lFollowerB = lFollowerB;

        lFollowerA.setInverted(true);
        lFollowerB.setInverted(true);
    }

    @Override
    public void moveToSetpoint(Measure<Velocity<Distance>> setpoint) {
        lead.set(setpoint.in(MetersPerSecond));
        rFollower.set(setpoint.in(MetersPerSecond));
        lFollowerA.set(setpoint.in(MetersPerSecond));
        lFollowerB.set(setpoint.in(MetersPerSecond));
    }

    @Override
    public double heightFromBase() {
        return lead.getPosition().getValue() * rotationFactor;
    }
}
