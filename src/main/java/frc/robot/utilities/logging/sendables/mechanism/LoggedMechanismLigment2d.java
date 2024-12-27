package frc.robot.utilities.logging.sendables.mechanism;

import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.HashMap;

public class LoggedMechanismLigment2d implements Loggable {
  private double angle;
  private double length;
  private HashMap<String, LoggedMechanismLigment2d> ligments;

  public LoggedMechanismLigment2d(double length, double angle) {
    this.angle = angle;
    this.length = length;
    ligments = new HashMap<>();
  }

  public void setAngle(double angle) {
    this.angle = angle;
  }

  public void setLength(double length) {
    this.length = length;
  }

  public void append(String name, LoggedMechanismLigment2d ligment) {
    ligments.put(name, ligment);
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, ".type", "line");
    HoundLog.log(path, "angle", angle);
    HoundLog.log(path, "color", "#EB8934");
    HoundLog.log(path, "length", length);
    HoundLog.log(path, "weight", 10);
    for (String ligmentName : ligments.keySet()) {
      HoundLog.log(path, ligmentName, ligments.get(ligmentName));
    }
  }
}
