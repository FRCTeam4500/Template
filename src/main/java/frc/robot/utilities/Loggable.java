package frc.robot.utilities;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

public interface Loggable {
    public void logData(Logger logger, LogTable table);
    public String getTableName();
}
