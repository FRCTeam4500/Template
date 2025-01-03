package frc.robot.utilities.logging.sendables.mechanism;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.HashMap;

public class LoggedMechanism2d implements Loggable {
  private double[] dims;
  private HashMap<String, LoggedMechanismRoot2d> roots;
  private String color;

  public LoggedMechanism2d(double width, double height) {
    dims = new double[] {width, height};
    roots = new HashMap<>();
  }

  public LoggedMechanism2d(double width, double height, Color8Bit backgroundColor) {
    dims = new double[] {width, height};
    roots = new HashMap<>();
    color = backgroundColor.toHexString();
  }

  public LoggedMechanismRoot2d getRoot(String name, double x, double y) {
    if (roots.containsKey(name)) {
      return roots.get(name);
    }
    LoggedMechanismRoot2d root = new LoggedMechanismRoot2d(x, y);
    roots.put(name, root);
    return root;
  }

  public void setBackgroundColor(Color8Bit color) {
    this.color = color.toHexString();
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, ".type", "Mechanism2d");
    HoundLog.log(path, "backgroundColor", color);
    HoundLog.log(path, ".controllable", true);
    HoundLog.log(path, "dims", dims);
    for (String rootName : roots.keySet()) {
      HoundLog.log(path, rootName, roots.get(rootName));
    }
  }
}
