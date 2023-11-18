package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Drive Command for teleop. Supports 3 modes <p>
 * 
 * <strong>Angle-Centric</strong>: Moves relative to field, but holds a specified 
 * field relative angle if it is hit by another robot. 
 * Can turn to 0 or 180 degrees easily. <p>
 * 
 * <strong>Align-to-Target</strong>: Angle-Centric, but can't turn or move 
 * sideways relative to the robot. Auto moves sideways to align with
 * game pieces. <p>
 * 
 * <strong>Robot-Centric</strong>: Moves relative to robot.
 */
public class TeleopDriveCommand extends CommandBase {
	private SwerveDrive swerve;
	private CommandXboxController controller;
	private DriveMode driveMode;
	private Rotation2d targetAngle;
	private Rotation2d alignmentAngle;
	private DriveSens driveSens;
	private double sidewaysVelocity;
	private double forwardVelocity;
	private double rotationalVelocity;

	public TeleopDriveCommand(CommandXboxController xboxController) {
		swerve = SwerveDrive.getInstance();
		controller = xboxController;
		addRequirements(swerve);
	}

    @Override
    public void initialize() {
        driveMode = DriveMode.AngleCentric;
		targetAngle = swerve.getRobotAngle();
		alignmentAngle = swerve.getRobotAngle();
		driveSens = DriveSens.Fast;
		Shuffleboard.getTab("Display").addString("Drive Mode", () -> driveMode.name());
		Shuffleboard.getTab("Display").addDouble("Target Angle Degrees ", () -> targetAngle.getDegrees());
    }

	@Override
	public void execute() {
		double rightX = -controller.getRightX();
		double rightY = -controller.getRightY();
		double leftX = -controller.getLeftX();
		double leftY = -controller.getLeftY();

		if (Math.abs(rightY) > 0.5) {
			targetAngle = Rotation2d.fromDegrees(90 - 90 * Math.signum(rightY));
		}
        targetAngle = Rotation2d.fromDegrees(targetAngle.getDegrees() + rightX * driveSens.rotational);

        sidewaysVelocity = leftX * driveSens.forward;
        forwardVelocity = leftY * driveSens.sideways;
		rotationalVelocity = rightX * driveSens.rotational;

		switch (driveMode) {
			case RobotCentric:
				swerve.driveRobotCentric(
					new ChassisSpeeds(
						forwardVelocity, 
						sidewaysVelocity,
						rotationalVelocity
					)
                );
				break;
			case AngleCentric:
				swerve.driveAngleCentric(
                    forwardVelocity, 
                    sidewaysVelocity, 
                    targetAngle
                );
                break;
			case AlignToTarget:
				swerve.driveAlignToTarget(
					forwardVelocity, 
					sidewaysVelocity,
					alignmentAngle
				);
				break;
        }
	}

    public Command toggleSlowModeCommand() {
        return Commands.startEnd(
            () -> {
                driveSens = DriveSens.Slow;
            },
            () -> {
                driveSens = DriveSens.Fast;
            }
        );
    }

    public Command toggleRobotCentricCommand() {
        return Commands.startEnd(
            () -> driveMode = DriveMode.RobotCentric,
            () -> {
				driveMode = DriveMode.AngleCentric;
				targetAngle = swerve.getRobotAngle();
			}
        );
    }

	public Command toggleAlignToAngleCommand() {
		return Commands.startEnd(
			() -> {
				driveMode = DriveMode.AlignToTarget;
				alignmentAngle = swerve.getRobotAngle();
			}, 
			() -> {
				driveMode = DriveMode.AngleCentric;
				targetAngle = swerve.getRobotAngle();
			}
		);
	}

	public Command setTargetAngleCommand(Rotation2d newTarget) {
		return Commands.runOnce(
			() -> targetAngle = newTarget
		);
	}

    public Command resetGyroCommand() {
        return Commands.runOnce(
            () -> {
                targetAngle = new Rotation2d();
                swerve.resetRobotAngle(new Rotation2d());
            }
        );    
    }

	public static enum DriveMode {
		AngleCentric,
		RobotCentric,
		AlignToTarget
	}

	public static enum DriveSens {
		Fast(4, 4, 3.5),
		Slow(0.8, 0.8, 0.5);

		public double forward;
		public double sideways;
		public double rotational;
		public boolean slew;

		private DriveSens(double forward, double sideways, double rotational) {
			this.forward = forward;
			this.sideways = sideways;
			this.rotational = rotational;
		}
	}
}