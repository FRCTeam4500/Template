package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.GamePieceVision;
import frc.robot.utilities.ExtendedMath;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Superstructure {
    private static Superstructure instance;
    public static synchronized Superstructure getInstance() {
        if (instance == null) instance = new Superstructure();
        return instance;
    }

    private SwerveDrive swerve;
    private AprilTagVision tagVision;
    private GamePieceVision pieceVision;

    private Rotation2d targetAngle;
    private DriveMode driveMode;

    public Superstructure() {
        swerve = SwerveDrive.getInstance();
        tagVision = AprilTagVision.getInstance();
        pieceVision = GamePieceVision.getInstance();
        targetAngle = swerve.getRobotAngle();
        driveMode = DriveMode.AngleCentric;

        configurePathPlanner();

        Shuffleboard.getTab("Display").addString("Drive Mode", () -> driveMode.toString());
    }

    public void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
            swerve::getOdometryPose, 
            swerve::resetPose, 
            swerve::getChassisSpeeds, 
            swerve::driveRobotCentric, 
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0),
                new PIDConstants(5.0),
                MAX_LINEAR_SPEED_MPS,
                0.39878808909,
                new ReplanningConfig()
            ), 
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, 
            swerve
        );
        NamedCommands.registerCommand("Drive To Piece", driveToPieceCommand().withTimeout(1.5));
    }

    public Command angleCentricDriveCommand(CommandXboxController xbox) {
        return Commands.run(
            () -> {
                double coefficent = Math.max(1 - xbox.getLeftTriggerAxis(), 0.2);
                double forwardSens = MAX_FORWARD_SENSITIVITY * coefficent;
                double sidewaysSens = MAX_SIDEWAYS_SENSITIVITY * coefficent;
                double rotationalSens = MAX_ROTATIONAL_SENSITIVITY * coefficent;
                if (Math.abs(xbox.getRightY()) > 0.5){
                    targetAngle = Rotation2d.fromDegrees(90 - 90 * Math.signum(-xbox.getRightY()));
                }
                targetAngle = Rotation2d.fromDegrees(
                    targetAngle.getDegrees() - 
                    xbox.getRightX() * rotationalSens
                );
                swerve.driveAngleCentric(
                    -xbox.getLeftY() * forwardSens,
                    -xbox.getLeftX() * sidewaysSens,
                    targetAngle
                );
            }, swerve
        ).beforeStarting(() -> {
            targetAngle = swerve.getRobotAngle();
            driveMode = DriveMode.AngleCentric;
        });
    }

    public Command robotCentricDriveCommand(CommandXboxController xbox) {
        return Commands.run(() -> {
                double coefficent = Math.max(1 - xbox.getLeftTriggerAxis(), 0.2);
                double forwardSens = MAX_FORWARD_SENSITIVITY * coefficent;
                double sidewaysSens = MAX_SIDEWAYS_SENSITIVITY * coefficent;
                double rotationalSens = MAX_ROTATIONAL_SENSITIVITY * coefficent;

                swerve.driveRobotCentric(new ChassisSpeeds(
                    -xbox.getLeftY() * forwardSens,
                    -xbox.getLeftX() * sidewaysSens,
                    -xbox.getRightX() * rotationalSens
                ));
            }, swerve
        ).beforeStarting(() -> driveMode = DriveMode.RobotCentric);
    }

    public Command alignToTargetDriveCommand(CommandXboxController xbox) {
        return Commands.run(() -> {
                double coefficent = Math.max(1 - xbox.getLeftTriggerAxis(), 0.2);
                double forwardSens = MAX_FORWARD_SENSITIVITY * coefficent;
                double sidewaysSens = MAX_SIDEWAYS_SENSITIVITY * coefficent;

                swerve.driveAlignToTarget(
                    -xbox.getLeftY() * forwardSens,
                    -xbox.getLeftX() * sidewaysSens,
                    targetAngle
                );
            }, swerve
        ).beforeStarting(() -> {
            targetAngle = swerve.getRobotAngle();
            driveMode = DriveMode.AlignToTarget;
        });
    }

    public Command driveToTagCommand(Pose2d targetPose) {
        return Commands.run(
			() -> {
				Pose2d poseDif = tagVision.getRelativeTagPose(targetPose).relativeTo(targetPose);
				swerve.driveRobotCentric(
                    new ChassisSpeeds(
						poseDif.getX() * 2,
						poseDif.getY() * 2,
						-poseDif.getRotation().getRadians() * 5
					)
				);
			}, swerve
		);
    }

    public Command driveToPieceCommand() {
        return Commands.run(
            () -> {
                if (!pieceVision.seesPiece()) {
                    swerve.driveRobotCentric(new ChassisSpeeds());
                } else {
                    Rotation2d diff = pieceVision.getHorizontalOffset(new Rotation2d());
                    double area = pieceVision.getTakenArea(40);
                    swerve.driveRobotCentric(new ChassisSpeeds(
                        ExtendedMath.clamp(0, 1, 20 - area),
                        5 * diff.getRadians(), 
                        0
                    ));
                }
            }, swerve);
    }

    public Command resetGyroCommand() {
        return Commands.runOnce(
            () -> {
                swerve.zeroRobotAngle();
                targetAngle = new Rotation2d();
            }
        );
    }
}