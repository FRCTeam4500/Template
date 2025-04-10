package frc.robot.programs;

import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.utilities.logging.HoundLog;

public class LoggedRobot extends TimedRobot {
  public LoggedRobot() {
    HoundLog.setEnabled(true);
    HoundLog.setOptions(
        new DogLogOptions(
            () -> !DriverStation.isFMSAttached(), true, true, true, true, 1000, () -> false));
  }
}
