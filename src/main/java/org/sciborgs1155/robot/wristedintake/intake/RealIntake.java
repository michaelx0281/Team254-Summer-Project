package org.sciborgs1155.robot.wristedintake.intake;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/**
 * RealIntake
 */
public class RealIntake implements IntakeIO{
    TalonSRX right, left;
    TalonSRXConfiguration config;
    SupplyCurrentLimitConfiguration currentConfig 
        = new SupplyCurrentLimitConfiguration
            (true,
            30,
            0,
            0); //remember to set these to good values / link to constants later.
    public RealIntake() {
        right.setNeutralMode(NeutralMode.Brake);
        // right.set(TalonSRXControlMode.PercentOutput, 1);
        right.configSupplyCurrentLimit(currentConfig);
        right.setControlFramePeriod(ControlFrame.Control_4_Advanced, 2);
        right.setInverted(InvertType.valueOf("Master Intake Motor"));

        left.set(TalonSRXControlMode.Follower, right.getDeviceID());
    }
    @Override
    public void setVoltage(double voltage) {
        // TODO Auto-generated method stub
        right.set(TalonSRXControlMode.Position, voltage);
    }

    @Override
    public Measure<Velocity<Angle>> getSpeed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSpeed'");
    }
    
}