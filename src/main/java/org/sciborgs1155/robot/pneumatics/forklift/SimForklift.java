package org.sciborgs1155.robot.pneumatics.forklift;

import edu.wpi.first.networktables.BooleanEntry;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.pneumatics.PneumaticsIO;

public class SimForklift implements PneumaticsIO {
  private BooleanEntry entry;

  public SimForklift() {
    entry = Tuning.entry("/Pneumatics/SimForklift", true);
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
