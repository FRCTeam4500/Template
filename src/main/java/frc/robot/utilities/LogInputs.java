package frc.robot.utilities;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LogInputs implements LoggableInputs{
    private static LogInputs instance;
	private Loggable target;

	public static synchronized LogInputs getInstance() {
        return instance == null ? new LogInputs() : instance;
    }

    public void setLoggingTarget(Loggable target) {
        this.target = target;
    }

    public void toLog(LogTable table) {
        target.logData(Logger.getInstance(), table);
    }

    public void fromLog(LogTable table) {}
}
