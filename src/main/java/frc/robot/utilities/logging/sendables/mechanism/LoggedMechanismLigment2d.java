package frc.robot.utilities.logging.sendables.mechanism;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.HashMap;

public class LoggedMechanismLigment2d implements Loggable {
  private double angle;
  private double length;
  private double weight;
  private String color;
  private HashMap<String, LoggedMechanismLigment2d> ligments;

  public LoggedMechanismLigment2d(double length, double angle) {
    this(angle, length, new Color8Bit(235, 137, 52), 10);
  }

  public LoggedMechanismLigment2d(double length, double angle, Color8Bit color, double lineWeight) {
    setAngle(angle);
    setLength(length);
    setLineWeight(lineWeight);
    setColor(color);
    ligments = new HashMap<>();
  }

  public void setAngle(double angle) {
    this.angle = angle;
  }

  public void setLength(double length) {
    this.length = length;
  }

  public void setColor(Color8Bit color) {
    this.color = color.toHexString();
  }

  public void setLineWeight(double weight) {
    this.weight = weight;
  }

  public void append(String name, LoggedMechanismLigment2d ligment) {
    if (ligments.containsKey(name)) {
      throw new UnsupportedOperationException("Mechanism ligment names must be unique!");
    }
    ligments.put(name, ligment);
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, ".type", "line");
    HoundLog.log(path, "angle", angle);
    HoundLog.log(path, "color", color);
    HoundLog.log(path, "length", length);
    HoundLog.log(path, "weight", weight);
    for (String ligmentName : ligments.keySet()) {
      HoundLog.log(path, ligmentName, ligments.get(ligmentName));
    }
  }
}
