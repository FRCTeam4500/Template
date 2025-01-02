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

  public void setPosiiton(double x, double y) {
    this.x = x;
    this.y = y;
  }

  public LoggedMechanismLigment2d append(String name, LoggedMechanismLigment2d ligment) {
    if (ligments.containsKey(name)) {
      throw new UnsupportedOperationException("Mechanism ligment names must be unique!");
    }
    ligments.put(name, ligment);
    return ligment;
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
