package frc.robot.subsystems.messaging;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Loggable;

public class MessagingSystem extends SubsystemBase implements Loggable{
    private static MessagingSystem instance;
    private String message;
    private String newestMessage;
    private boolean isEnabled = false;

    private MessagingSystem() {
        message = "MESSAGES APPEAR BELOW";
    }

    public void addMessage(String message) {
        if(isEnabled) {
            newestMessage = message;
            this.message = this.message + "\n" + newestMessage;
        }
    }

    public void enableMessaging(boolean enable) {
        isEnabled = enable;
    }

    public void enableMessaging() {
        isEnabled = true;
    }

    public static synchronized MessagingSystem getInstance() {
		return instance == null ? new MessagingSystem() : instance;
	}

    @Override
    public void logData(Logger logger,LogTable table) {
        table.put("Message", newestMessage);
    }

    @Override
    public String getTableName() {
        return "Messaging System";
    }

    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Messages", () -> message, null);
    }
}
