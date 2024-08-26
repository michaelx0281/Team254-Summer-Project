package org.sciborgs1155.robot.pneumatics.forklift
;

import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.pneumatics.PneumaticsIO;

import edu.wpi.first.networktables.BooleanEntry;

public class SimForklift implements PneumaticsIO{

    BooleanEntry entry;

    public SimForklift() {
        Tuning.entry("/Pneumatics/Simforklift", true);
    } 
    
    @Override
    public void extend() {
        entry.accept(true);
    }

    @Override
    public void retract() {
        entry.accept(false);
    }

}