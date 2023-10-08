package frc.robot.autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Autonomous {

	private static Autonomous autonomous = null;

	SwerveDrive swerve;

	private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

	private Autonomous() {
		swerve = SwerveDrive.getInstance();
		configureAuto();
	}

	private void configureAuto() {
		autonChooser.setDefaultOption("No Auto", null);
		Shuffleboard.getTab("Display").add("Auto Route", autonChooser);
	}

	/**
	 * Returns the instance of the autonomous class. If the instance is null, it will create a new instance.
	 * @return
	 */
	public static synchronized Autonomous getInstance() {
		if (autonomous == null) {
			autonomous = new Autonomous();
		}
		return autonomous;
	}

	/**
	 * Returns the selected autonomous command.
	 * @return
	 */
	public Command getAutonCommand() {
		return autonChooser.getSelected();
	}
}
