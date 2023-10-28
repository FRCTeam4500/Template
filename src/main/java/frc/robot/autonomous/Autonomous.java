package frc.robot.autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class Autonomous {
	private static Autonomous instance;
	private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

	private Autonomous() {
		configureAuto();
	}

	private void configureAuto() {
		autonChooser.setDefaultOption("No Auto", null);
		Shuffleboard.getTab("Display").add("Auto Route", autonChooser);
	}

	public static synchronized Autonomous getInstance() {
		return instance == null ? new Autonomous() : instance;
	}

	public Command getAutonCommand() {
		return autonChooser.getSelected();
	}
}
