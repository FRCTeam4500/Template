package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Limelight;
import frc.robot.utilities.Loggable;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase implements Loggable {
	private static Vision instance;
	private Limelight aprilTagLimelight;
	private Limelight gamePieceLimelight;

	//TODO: Change these
	private double GAMEPIECE_LIMELIGHT_HEIGHT_METERS = 0.232;
	private double GAMEPIECE_HALF_HEIGHT_METERS = 0.16;
	private Rotation2d GAMEPIECE_LIMELIGHT_ANGLE = Rotation2d.fromDegrees(-12);

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
		table.put("Tag ID", getTagId(0));
        table.put("Sees tag", seesTag());
        table.put("Sees gamepiece", seesGamePiece());
		Logger.getInstance().recordOutput("Vision Odometry", getRobotPose(new Pose2d()));
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

	public Translation2d getTranslation(Translation2d defaultTranslation) {
		if (!seesGamePiece()) return defaultTranslation;
		double forwardDistance = 
			(GAMEPIECE_LIMELIGHT_HEIGHT_METERS - GAMEPIECE_HALF_HEIGHT_METERS) / 
			Math.tan(
				GAMEPIECE_LIMELIGHT_ANGLE.plus(
					getVerticalOffset(new Rotation2d())
				).getRadians()
			);
		return new Translation2d(
			forwardDistance,
			forwardDistance * Math.tan(
				getVerticalOffset(new Rotation2d())
				.getRadians()
			)
		);
	}

	public int getTagId(int defaultID) {
		return aprilTagLimelight.getTargetTagId().orElse(defaultID);
	}

	public Pose2d getRobotPose(Pose2d defaultPose) {
		return getRobotPose(defaultPose, DriverStation.getAlliance());
	}

	public Pose2d getRobotPose(Pose2d defaultPose, Alliance poseOrigin) {
		return aprilTagLimelight
			.getRobotPoseToAlliance(poseOrigin)
			.orElse(defaultPose);
	}

    public Pose2d getRelativeTagPose(Pose2d defaultPose) {
        if (!seesTag()) return defaultPose;
		return getRobotPose(new Pose2d(), Alliance.Blue)
			.relativeTo(getTagPose(getTagId(0)));
    }

	public Rotation2d getHorizontalOffset(Rotation2d defaultRotation) {
		return gamePieceLimelight
			.getHorizontalOffsetFromCrosshair()
			.orElse(defaultRotation);
	}

	public Rotation2d getVerticalOffset(Rotation2d defaultRotation) {
		return gamePieceLimelight
			.getVerticalOffsetFromCrosshair()
			.orElse(defaultRotation);
	}

	public double getTakenArea(double defaultArea) {
		return gamePieceLimelight
			.getTargetArea()
			.orElse(defaultArea);
	}

	public Rotation2d getSkew(Rotation2d defaultSkew) {
		return gamePieceLimelight
			.getSkew()
			.orElse(defaultSkew);
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
