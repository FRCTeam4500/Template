package frc.robot.commands.complexCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DriveController;
import frc.robot.Constants.EnumConstants.DriveMode;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * A swerve command with support for two swerve control modes:
 * <p>
 * Field-Centric:
 * Robot moves relative to the field's axes.
 * When pushing the joystick forward, the robot moves down the field, no matter which way it is facing
 * (Actually, it moves in whatever direction is zeroed to, this just assumes that the gyro is zeroed down the field)
 * <p>
 * Robot-Centric:
 * The robot moves relative to itself.
 * When pushing the joystick foward, the robot moves in whatever direction it is facing.
 * For our purposes, the front of the robot is the intake side.
 */
public class SwerveDriveCommand extends CommandBase {
	private SwerveDrive swerve;
	private CommandXboxController controller;

	public DriveMode driveMode;

	public SlewRateLimiter xLimiter = new SlewRateLimiter(1.75);
	public SlewRateLimiter yLimiter = new SlewRateLimiter(1.75);

	private PIDController angleController;
	private boolean doSlew;

	private double xSens;
	private double ySens;
	private double zSens;
	
	private double xSpeed;
	private double ySpeed;
	private double zSpeed;

	private double targetAngle = 0;

	public SwerveDriveCommand(DriveController xboxController) {
		swerve = SwerveDrive.getInstance();
		controller = xboxController;
		driveMode = DriveMode.AngleCentric;
		angleController = new PIDController(1, 0, 0);
		angleController.enableContinuousInput(-Math.PI, Math.PI);
		fastSpeed();
		addRequirements(swerve);
	}

	@Override
	public void execute() {
		double triggerTotal = controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
		double rightX = -controller.getRightX();
		double rightY = -controller.getRightY();
		double rightAngle = Math.toDegrees(Math.atan(rightY / rightX));
		double rightMagnitude = Math.sqrt((Math.pow(rightX, 2)) + Math.pow(rightY, 2));
		double leftX = -controller.getLeftX();
		double leftY = -controller.getLeftY();

		if (rightMagnitude > 0.1) {
			targetAngle = rightAngle;
		}
		if (Math.abs(triggerTotal) > 0.1) {
			targetAngle += triggerTotal * Math.toDegrees(zSens);
		}

		if(doSlew) {
			xSpeed = xLimiter.calculate(leftX) * xSens;
			ySpeed = yLimiter.calculate(leftY) * ySens;
		} else {
			xSpeed = leftX * xSens;
			ySpeed = leftY * ySens;
		}
		zSpeed = rightX * zSens;

		switch (driveMode) {
			case RobotCentric:
				moveRobotCentric(xSpeed, ySpeed, zSpeed);
				break;
			case AngleCentric:
				moveAngleCentric(xSpeed, ySpeed);	
				break;
		}
	}

	private void moveRobotCentric(double x, double y, double w) {
		swerve.driveRobotCentric(y, x, w);
	}

	private void moveAngleCentric(double xSpeed, double ySpeed) {
		double wSpeed =
			3.75 *
			angleController.calculate(
				swerve.getRobotAngle(),
				Math.toRadians(targetAngle)
			);
		swerve.driveFieldCentric(ySpeed, xSpeed, wSpeed);
	}

	public void fastSpeed() {
		xSens = 4;
		ySens = 4;
		zSens = 3.5;
		doSlew = true;
		MessagingSystem.getInstance().addMessage("Swerve -> Robot Speed -> Fast");
	}
	
	public void slowSpeed() {
		xSens = .8;
		ySens = .8;
		zSens = .5;
		doSlew = false;
		MessagingSystem.getInstance().addMessage("Swerve -> Robot Speed -> Slow");
	}

	public void setTargetAngle(double angle) {
		targetAngle = angle;
	}

	public void switchDriveMode() {
		if (driveMode == DriveMode.RobotCentric) {
			setDriveMode(DriveMode.AngleCentric);
		} else {
			setDriveMode(DriveMode.RobotCentric);
		}
	}

	public void setDriveMode(DriveMode newDriveMode) {
		driveMode = newDriveMode;
		MessagingSystem.getInstance().addMessage("Drive Mode is now " + newDriveMode.name());
	}

	public void initSendable(SendableBuilder builder) {
		builder.addStringProperty("Drive Mode", () -> driveMode.name(), null);
		builder.addDoubleProperty("Target Angle: ", () -> targetAngle, null);
	}
}
