package frc.robot.utilities;

import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import java.util.TimerTask;

import org.littletonrobotics.junction.Logger;

public class LogSubsystemInputsTask extends TimerTask {
	private LogInputs inputHandler = LogInputs.getInstance();
	private Loggable[] thingsToLog = {
		SwerveDrive.getInstance(),
		Vision.getInstance(),
		MessagingSystem.getInstance()
	};

	@Override
	public void run() {
		for (int i = 0; i < thingsToLog.length; i++) {
			inputHandler.setLoggingTarget(thingsToLog[i]);
			Logger.getInstance().processInputs(thingsToLog[i].getTableName(), inputHandler);
		}
	}
}
