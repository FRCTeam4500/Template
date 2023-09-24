package frc.robot.autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.baseCommands.ResetGyroCommand;
import frc.robot.subsystems.swerve.SwerveDrive;

import java.util.HashMap;

public class Autonomous {

	private static Autonomous autonomous = null;

	SwerveDrive swerve;

	/** A hash map containing the commands the robot will use in auto <p> These commands can be accessed by putting the cooresponding string key into the .get() method
	 * <p> Example: {@code autoCommandMap.get("zero");}
	 */
	public static final HashMap<String, Command> autoCommandMap = new HashMap<>();

	private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

	private Autonomous() {
		swerve = SwerveDrive.getInstance();
		configureAuto();
	}

	private void configureAuto() {
		autoCommandMap.put("resetGyro", new ResetGyroCommand());
		autoCommandMap.put("reverseGyro", new ResetGyroCommand(180));
		
		autonChooser.setDefaultOption("No Auto", null);

		Shuffleboard.getTab("Auto").add("Auto Routes", autonChooser);
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
