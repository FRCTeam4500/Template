package frc.robot.subsystems.messaging;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class MessagingSystem extends SubsystemBase implements LoggableInputs {
	private static MessagingSystem instance;
	private StringBuilder messages;
	private boolean isEnabled;

	private MessagingSystem() {
		messages = new StringBuilder("MESSAGES APPEAR BELOW");
        isEnabled = false;
	}

	public void addMessage(String message) {
		if (isEnabled) {
			messages.append("\n").append(message);
		}
	}

	public void setMessagingState(boolean enable) {
		isEnabled = enable;
	}

	public static synchronized MessagingSystem getInstance() {
		if (instance == null) instance = new MessagingSystem();
        return instance;
	}

	@Override
	public void toLog(LogTable table) {
		table.put("Message", messages.toString());
	}

	@Override
	public void fromLog(LogTable table) {}
}
