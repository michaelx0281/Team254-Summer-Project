package org.sciborgs1155.robot.pneumatics.hanger;

import org.sciborgs1155.robot.pneumatics.PneumaticsIO;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class RealHanger implements PneumaticsIO {
Compressor compressor;
    Solenoid solenoid;

    public RealHanger() {
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();

    
    }

    @Override
    public void extend() {
        solenoid.set(true);
        
    }

    @Override
    public void retract() {
        solenoid.set(false);
    }
}