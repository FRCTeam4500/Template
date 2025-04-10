package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.orchestra.Orc;
import frc.robot.utilities.StopTilting;
import frc.robot.utilities.logging.Loggable;

/**
 * A class that holds together the top half of our robot. Basically everything except the
 * drivetrain. It exposes command factories which combine the various subsystems
 */
public class Superstructure implements Loggable {
  // Create objects for all non-drivebase subsystems
 
  public Superstructure() {
    StopTilting.setupSuperstructure(
        new Transform3d[] {

        },
        new double[] {});
  }

  public void log(String path) {
    // Call log() methods for contained subsystems
    StopTilting.updateCenterOfMass(
        new Transform3d[] {});
  }

  public Command sing() {
    return Orc.startSinging();
  }

  public Command stopSinging() {
    return Orc.stopSinging();
  }

  public Command stow() {
    return Commands.none();
  }
}