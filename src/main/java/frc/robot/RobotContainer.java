package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.Autonomous;
import frc.robot.subsystems.messaging.MessagingSystem;

public class RobotContainer {
	private final Autonomous autonomous;
	private final MessagingSystem messaging;
	private Command autoCommand;

	public RobotContainer() {
		DriveController.getInstance();
		autonomous = Autonomous.getInstance();
		messaging = MessagingSystem.getInstance();
	}

	public void autonomousInit() {
		messaging.enableMessaging();
		messaging.addMessage("Auto Started");
		autoCommand = autonomous.getAutonCommand();
		if (autoCommand != null) {
			autoCommand.schedule();
		} else {
			messaging.addMessage("No Auto Command Selected");
		}
	}

	public void teleopInit() {
		messaging.enableMessaging();
		messaging.addMessage("Teleop Started");
		if (autoCommand != null) {
			autoCommand.cancel();
		}
	}
}
