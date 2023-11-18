package frc.robot.utilities;

import org.littletonrobotics.junction.LogTable;

public interface Loggable {
	public void logData(LogTable table);
	public String getTableName();
}
