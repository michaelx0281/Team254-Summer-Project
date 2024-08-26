package org.sciborgs1155.robot.wristedintake.wrist;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Constants.RPS_TO_RADIANS_PER_SECOND;
import static org.sciborgs1155.robot.wristedintake.wrist.WristConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/**
 * RealWrist
 */
public class RealWrist implements WristIO{

    TalonFX talon = new TalonFX(0, "rio");
    TalonFXConfigurator configurator = talon.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();

    public RealWrist() {
        configurator.refresh(config);
        /* Adding all necessary adjustments to the configuration */
        config  // TODO check more about everything that can possibly be configured later.
            .withCurrentLimits(
                config.CurrentLimits
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(30)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(30)
            )
            .withMotorOutput(
                config.MotorOutput
                    .withInverted(InvertedValue.valueOf("IntakeMaster"))
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withFeedback(
                config.Feedback
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withFeedbackRotorOffset(ROTOR_OFFSET) //TODO make these constants
                    .withRotorToSensorRatio(ROTOR_TO_SENSOR_RATIO)
            );
        DutyCycleOut req = new DutyCycleOut(0);

        /* Setting right motor as master and having the left follow, inverted, with 100% of voltage supplied */
        talon.setControl(req.withOutput(1));
    }

    @Override
    public void setVoltage(double volts) {
        talon.setVoltage(volts);
    }

    @Override
    public Measure<Angle> getPositionRadians() {
        return Radians.of(talon.getPosition().getValueAsDouble() * RPS_TO_RADIANS_PER_SECOND);
    }

    @Override
    public Measure<Velocity<Angle>> getSpeed() {
        return RadiansPerSecond.of(talon.getVelocity().getValueAsDouble() * RPS_TO_RADIANS_PER_SECOND);
    }

    
}