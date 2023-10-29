package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.baseCommands.CancellationCommand;
import frc.robot.commands.complexCommands.SwerveDriveCommand;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class DriveController extends CommandXboxController {
	private static DriveController instance;
	private SwerveDriveCommand swerveCommand;
	private final Trigger switchDriveModeButton = this.x();
	private final Trigger resetGyroButton = this.a();
	private final Trigger slowModeButton = this.leftBumper();
	private final Trigger cancelButton = this.start();

	private DriveController() {
		super(JoystickConstants.DRIVER_PORT);
		setButtons();
		addToShuffleBoard();
	}

	public static synchronized DriveController getInstance() {
		return instance == null ? new DriveController() : instance;
	}

	private void setButtons() {
		swerveCommand = new SwerveDriveCommand(this);
		SwerveDrive.getInstance().setDefaultCommand(swerveCommand);

		switchDriveModeButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.switchDriveMode()));

		resetGyroButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.resetGyro(0)));

		slowModeButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.slowSpeed()));
		slowModeButton.toggleOnFalse(new InstantCommand(() -> swerveCommand.fastSpeed()));

		cancelButton.toggleOnTrue(new CancellationCommand());
	}

	private void addToShuffleBoard() {
		Shuffleboard.getTab("Messaging").add("Messaging System", MessagingSystem.getInstance());
		Shuffleboard.getTab("Swerve").add("Swerve", SwerveDrive.getInstance());
		Shuffleboard.getTab("Vision").add("Vision", Vision.getInstance());
		Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
	}
}
