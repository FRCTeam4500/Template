package frc.robot.utilities.logging.sendables.mechanism;

import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.HashMap;

public class LoggedMechanismRoot2d implements Loggable {
  private double x;
  private double y;
  private HashMap<String, LoggedMechanismLigment2d> ligments;

  protected LoggedMechanismRoot2d(double x, double y) {
    this.x = x;
    this.y = y;
    ligments = new HashMap<>();
  }

  public void append(String name, LoggedMechanismLigment2d ligment) {
    ligments.put(name, ligment);
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "x", x);
    HoundLog.log(path, "y", y);
    for (String ligmentName : ligments.keySet()) {
      HoundLog.log(path, ligmentName, ligments.get(ligmentName));
    }
  }
}
