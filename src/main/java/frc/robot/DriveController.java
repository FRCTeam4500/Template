package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.baseCommands.CancellationCommand;
import frc.robot.commands.baseCommands.ResetGyroCommand;
import frc.robot.commands.complexCommands.AutoAlignRotationalCommand;
import frc.robot.commands.complexCommands.AutoDrivetoCommand;
import frc.robot.commands.complexCommands.SwerveDriveCommand;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class DriveController extends CommandXboxController {

	SwerveDriveCommand swerveCommand;

	private static DriveController instanceDriveController = null;

	private final Trigger switchDriveModeButton = this.x();
	private final Trigger resetGyroButton = this.a();
	private final Trigger slowModeButton = this.leftBumper();
	private final Trigger alignButton = this.y();
	private final Trigger cancelButton = this.start();

	private DriveController() {
		super(JoystickConstants.DRIVER_PORT);
		setButtons();
		addToShuffleBoard();
	}

	public static synchronized DriveController getInstance() {
		if (instanceDriveController == null) {
			instanceDriveController = new DriveController();
		}
		return instanceDriveController;
	}

	public void setButtons() {
		swerveCommand = new SwerveDriveCommand(this);
		SwerveDrive.getInstance().setDefaultCommand(swerveCommand);

		switchDriveModeButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.switchControlMode()));

		resetGyroButton.toggleOnTrue(new ResetGyroCommand());

		slowModeButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.slowSpeed()));
		slowModeButton.toggleOnFalse(new InstantCommand(() -> swerveCommand.fastSpeed()));

		alignButton.toggleOnTrue(
			new SequentialCommandGroup(
				new AutoAlignRotationalCommand(0, 1, 2),
				new AutoDrivetoCommand(0),
				new AutoAlignRotationalCommand(0, 1, 2)
			)
		);

		cancelButton.toggleOnTrue(new CancellationCommand());
	}

	public void addToShuffleBoard() {
		Shuffleboard.getTab("Messaging").add("Messaging System", MessagingSystem.getInstance());
		Shuffleboard.getTab("Swerve").add("Swerve", SwerveDrive.getInstance());
		Shuffleboard.getTab("Vision").add("Vision", Vision.getInstance());
		Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);

	}
}
