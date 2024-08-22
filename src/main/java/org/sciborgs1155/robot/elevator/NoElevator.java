package org.sciborgs1155.robot.elevator;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public class NoElevator implements ElevatorIO {

    @Override
    public void setVoltage(Measure<Voltage> volts) {}

    @Override
    public double heightFromBase() {
        return 0;
    }
    
}