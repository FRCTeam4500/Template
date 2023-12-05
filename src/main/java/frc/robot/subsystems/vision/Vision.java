package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
	private final double GAMEPIECE_LIMELIGHT_HEIGHT_METERS = 0.232;
	private final double GAMEPIECE_HALF_HEIGHT_METERS = 0.16;
	private final Rotation2d GAMEPIECE_LIMELIGHT_ANGLE = Rotation2d.fromDegrees(-12);

	private Vision() {
		aprilTagLimelight = new Limelight("limelight-hehehe");
		gamePieceLimelight = new Limelight("limelight-haha");
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

    public Optional<Pose2d> getRelativeTagPose() {
        if (!seesTag()) return Optional.empty();
        return Optional.of(
            getRobotPose(Alliance.Blue).orElse(new Pose2d())
                .relativeTo(getTagPose(getTagId().orElse(0)))
        );
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

    private Pose2d getTagPose(int tagId) {
        Rotation2d tagRotation = Rotation2d.fromDegrees(tagId > 4 ? 0 : 180);
        Translation2d tagTranslation = new Translation2d();
        double longOffset = 16.4846 / 2;
        double shortOffset = 8.1026 / 2;
        switch (tagId) {
            case 1:
                tagTranslation = new Translation2d(
                    7.24310 + longOffset,
                    -2.93659 + shortOffset
                );
                break;
            case 2:
                tagTranslation = new Translation2d(
                    7.24310 + longOffset,
                    -1.26019 + shortOffset
                );
                break;
            case 3:
                tagTranslation = new Translation2d(
                    7.24310 + longOffset,
                    0.41621 + shortOffset
                );
                break;
            case 4:
                tagTranslation = new Translation2d(
                    7.90832 + longOffset,
                    2.74161 + shortOffset
                );
                break;
            case 5:
                tagTranslation = new Translation2d(
                    -7.90832 + longOffset,
                    2.74161 + shortOffset
                );
                break;
            case 6:
                tagTranslation = new Translation2d(
                    -7.24310 + longOffset,
                    0.41621 + shortOffset
                );
                break;
            case 7:
                tagTranslation = new Translation2d(
                    -7.24310 + longOffset,
                    -1.26019 + shortOffset
                );
                break;
            case 8:
                tagTranslation = new Translation2d(
                    -7.24310 + longOffset,
                    -1.26019 + shortOffset
                );
                break;
            default:
                tagTranslation = new Translation2d();
                break;
        }
        return new Pose2d(tagTranslation, tagRotation);  
    }
}
