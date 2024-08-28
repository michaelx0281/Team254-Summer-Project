package org.sciborgs1155.robot.pneumatics.hanger;

import edu.wpi.first.networktables.BooleanEntry;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.pneumatics.PneumaticsIO;

public class SimHanger implements PneumaticsIO {
  private BooleanEntry entry;

  public SimHanger() {
    entry = Tuning.entry("/Pneumatics/SimHanger", true);
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
