package frc.robot;

import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import frc.robot.utilities.logging.sendables.mechanism.LoggedMechanism2d;

/**
 * A class that holds together the top half of our robot. Basically everything except the
 * drivetrain. It exposes command factories which combine the various subsystems to preform tasks
 */
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
    HoundLog.log(name, "Robot Mech", robotMech);
  }

  // Put Command Factories Here
}
