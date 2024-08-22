package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

// wtf is this?

public class RealElevator implements ElevatorIO {

  TalonFX lead, rFollower, lFollowerA, lFollowerB;
  TalonFXConfigurator leadConfig = lead.getConfigurator();
  TalonFXConfiguration rfxConfigs = new TalonFXConfiguration();
  TalonFXConfiguration lfxConfigs;

  public RealElevator() {
    lead.setPosition(0);
    leadConfig.refresh(rfxConfigs);

    rfxConfigs.withCurrentLimits(rfxConfigs.CurrentLimits
      .withSupplyCurrentLimit(30)
      .withSupplyCurrentLimitEnable(true)
      .withStatorCurrentLimit(20)
      .withStatorCurrentLimitEnable(true))
    .withFeedback(rfxConfigs.Feedback
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
      .withFeedbackRotorOffset(ROTOR_OFFSET))
    .withMotorOutput(rfxConfigs.MotorOutput
      .withNeutralMode(NeutralModeValue.Brake)); 
    
    // lfxConfigs = rfxConfigs.withMotorOutput(rfxConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
    final DutyCycleOut request = new DutyCycleOut(0);
    lead.setControl(request.withOutput(1.0));

    rFollower.setControl(new Follower(lead.getDeviceID(), false));
    lFollowerA.setControl(new Follower(lead.getDeviceID(), true));
    lFollowerB.setControl(new Follower(lead.getDeviceID(), true));
    
  }

  // The talons (there were 4 implemented in the CheesyPuffs' code)

  private Measure<Distance> height = Meters.of(0.0);
  //figure out the configuration class methods to let the talons follow the lead...
  @Override
  public void setVoltage(Measure<Voltage> volts) {
    lead.set(volts.in(Volts));
    rFollower.set(volts.in(Volts));
    lFollowerA.set(volts.in(Volts));
    lFollowerB.set(volts.in(Volts));
  }

  @Override
  public double heightFromBase() {
    return lead.getPosition().getValue() * rotationFactor;
  }
}
