package frc.robot;

import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import frc.robot.utilities.logging.sendables.mechanism.LoggedMechanism2d;

public class Superstructure implements Loggable {
  // Create objects for all non-drivebase subsystems
  private LoggedMechanism2d robotMech;

  public Superstructure() {
    robotMech = new LoggedMechanism2d(1.5, 1.5);
    configureMech();
  }

  private void configureMech() {
    // Append subsystem mechs
  }

  public void log(String name) {
    // Call log() methods for contained subsystems
    HoundLog.log("Robot Mech", robotMech);
  }

  // Put Command Factories Here
}
