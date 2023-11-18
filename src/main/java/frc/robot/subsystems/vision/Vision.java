package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Limelight;
import frc.robot.utilities.Loggable;

import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase implements Loggable {
	private static Vision instance;
	private Limelight aprilTagLimelight;
	private Limelight gamePieceLimelight;

	// TODO: Change these!
	private final double GAMEPIECE_LIMELIGHT_HEIGHT_METERS = 0;
	private final double GAMEPIECE_HALF_HEIGHT_METERS = 0;
	private final Rotation2d GAMEPIECE_LIMELIGHT_ANGLE = Rotation2d.fromDegrees(0);

	private Vision() {
		aprilTagLimelight = new Limelight("limelight-hehehe");
		gamePieceLimelight = new Limelight("limelight-haha");
		Shuffleboard.getTab("Display").addDouble(
			"Horizontal Offset", 
			() -> getGamePieceHorizontalOffset().orElse(new Rotation2d()).getDegrees()
		);
		Shuffleboard.getTab("Display").addDouble(
			"Forward Distance", 
			() -> getGamePieceTranslation().orElse(new Translation2d()).getX()
		);
		Shuffleboard.getTab("Display").addDouble(
			"Sideways Distance",
			() -> getGamePieceTranslation().orElse(new Translation2d()).getY()
		);
	}

	public static synchronized Vision getInstance() {
		if (instance == null) instance = new Vision();
		return instance;
	}

	@Override
	public void logData(LogTable table) {
		table.put("Tag ID", getTagId().orElse(0));
        table.put("Sees tag", seesTag());
        table.put("Sees gamepiece", seesGamePiece());
		Logger.getInstance().recordOutput("Vision Odometry", getRobotPose().orElse(new Pose2d()));
	}

	@Override
	public String getTableName() {
		return "Vision";
	}

	public Limelight getAprilTageLimelight() {
		return aprilTagLimelight;
	}

	public Limelight getGamePieceLimelight() {
		return gamePieceLimelight;
	}

	public boolean seesTag() {
		return aprilTagLimelight.hasValidTargets();
	}

	public boolean seesGamePiece() {
		return gamePieceLimelight.hasValidTargets();
	}

	public Optional<Translation2d> getGamePieceTranslation() {
		if (!seesGamePiece()) return Optional.empty();
		double forwardDistance = 
			(GAMEPIECE_LIMELIGHT_HEIGHT_METERS - GAMEPIECE_HALF_HEIGHT_METERS) / 
			Math.tan(
				GAMEPIECE_LIMELIGHT_ANGLE.plus(
					getGamePieceVerticalOffset().orElse(new Rotation2d())
				).getRadians()
			);
		return Optional.of(
			new Translation2d(
				forwardDistance,
				forwardDistance * Math.tan(
					getGamePieceVerticalOffset().orElse(new Rotation2d())
					.getRadians()
				)
			)	
		);
	}

	public Optional<Integer> getTagId() {
		if (!seesTag()) return Optional.empty();
		return Optional.of(aprilTagLimelight.getTargetTagId());
	}

	public Optional<Pose2d> getRobotPose() {
		return getRobotPose(DriverStation.getAlliance());
	}

	public Optional<Pose2d> getRobotPose(Alliance poseOrigin) {
		if (!seesTag()) return Optional.empty();
		return Optional.of(aprilTagLimelight.getRobotPoseToAlliance(poseOrigin));
	}

	public Optional<Pose2d> getRelativeTargetPose() {
		if (!seesTag()) return Optional.empty();
		return Optional.of(aprilTagLimelight.getTargetPoseToRobot());
	}

	public Optional<Rotation2d> getGamePieceHorizontalOffset() {
		if (!seesGamePiece()) return Optional.empty();
		return Optional.of(gamePieceLimelight.getHorizontalOffsetFromCrosshair());
	}

	public Optional<Rotation2d> getGamePieceVerticalOffset() {
		if (!seesGamePiece()) return Optional.empty();
		return Optional.of(gamePieceLimelight.getVerticalOffsetFromCrosshair());
	}

	public Optional<Double> getGamePieceTakenArea() {
		if (!seesGamePiece()) return Optional.empty();
		return Optional.of(gamePieceLimelight.getTargetArea());
	}

	public Optional<Rotation2d> getGamePieceSkew() {
		if (!seesGamePiece()) return Optional.empty();
		return Optional.of(gamePieceLimelight.getSkew());
	}
}
