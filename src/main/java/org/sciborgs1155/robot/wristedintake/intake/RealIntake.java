package org.sciborgs1155.robot.wristedintake.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.wristedintake.intake.IntakeConstants.*;

import org.sciborgs1155.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
// import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/** RealIntake */
public class RealIntake implements IntakeIO {
  private static final Time Second = null;
  // Using TalonFX and CTRE 6 implementations
  TalonFX right = new TalonFX(0, "rio");
  TalonFX left = new TalonFX(1, "rio");
  TalonFXConfigurator configurator = right.getConfigurator();
  TalonFXConfiguration config = new TalonFXConfiguration();

  /* Sets up configuration of hardware */
  public RealIntake() {
    configurator.refresh(config);
    /* Adding all necessary adjustments to the configuration */
    config // TODO check more about everything that can possibly be configured later.
        .withCurrentLimits(
            config.CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(30))
        .withMotorOutput(
            config.MotorOutput.withInverted(InvertedValue.valueOf("IntakeMaster"))
                .withNeutralMode(NeutralModeValue.Brake))
        .withFeedback(
            config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withFeedbackRotorOffset(ROTOR_OFFSET)
                .withRotorToSensorRatio(ROTOR_TO_SENSOR_RATIO));
    DutyCycleOut req = new DutyCycleOut(0);

    /* Setting right motor as master and having the left follow, inverted, with 100% of voltage supplied */
    right.setControl(req.withOutput(1));
    left.setControl(new Follower(right.getDeviceID(), true));
  }

  /* Sets right (lead motor) voltage. */
  @Override
  public void setVoltage(double voltage) {
    right.setVoltage(voltage);
  }

  /* Gets velocity of right rotor in rads/sec. */
  @Override
  public Measure<Velocity<Angle>> getSpeed() {
    return RadiansPerSecond.of(
        right.getRotorVelocity().getValueAsDouble() * RPS_TO_RADIANS_PER_SECOND);
  }

  @Override
  public Measure<Voltage> voltage() {
    return Volts.of(right.getMotorVoltage().getValueAsDouble());
  }

  @Override
  public Measure<Velocity<Velocity<Angle>>> getAccel() {
    return RadiansPerSecond.per(Second).of(right.getAcceleration().getValueAsDouble() * Constants.RPS_TO_RADIANS_PER_SECOND);
  }

  @Override
  public Measure<Angle> getPositionRads() {
    return Radians.of(right.getRotorPosition().getValueAsDouble() * Constants.RPS_TO_RADIANS_PER_SECOND);
  }
}
