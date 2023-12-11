package frc.robot.utilities.logging;

import org.littletonrobotics.junction.LogTable;

public interface Loggable {
	public void logData(LogTable table);
	public String getTableName();
}
