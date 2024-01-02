package frc.robot.subsystems.vision;


import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Limelight;
import frc.robot.utilities.ExtendedMath;

public class AprilTagVision extends SubsystemBase implements LoggableInputs {
    private static AprilTagVision instance;
    public static synchronized AprilTagVision getInstance() {
        if (instance == null) instance = new AprilTagVision();
        return instance;
    }

    private Limelight limelight;

    public AprilTagVision() {
        limelight = new Limelight("limelight-hehehe");
    }

    public boolean seesTag() {
        return limelight.hasValidTargets();
    }

    public int getTagId(int defaultId) {
        return limelight.getTargetTagId().orElse(defaultId);
    }

    public Pose2d getRobotPose(Pose2d defaultPose) {
		return getRobotPose(defaultPose, DriverStation.getAlliance());
	}

	public Pose2d getRobotPose(Pose2d defaultPose, Alliance poseOrigin) {
		return limelight
			.getRobotPoseToAlliance(poseOrigin)
			.orElse(defaultPose);
	}

    public Pose2d getRelativeTagPose(Pose2d defaultPose) {
        if (!seesTag()) return defaultPose;
		Pose2d backwardsPose = getRobotPose(new Pose2d(), Alliance.Blue)
			.relativeTo(getTagPose(getTagId(0)));
        return new Pose2d(
            backwardsPose.getTranslation(), 
            ExtendedMath.wrapRotation2d(backwardsPose.getRotation()
                .plus(Rotation2d.fromDegrees(180)))
        );
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

    @Override
    public void toLog(LogTable table) {
        table.put("Tag ID", getTagId(0));
        table.put("Sees tag", seesTag());
        Logger.getInstance().recordOutput(
            "Robot Pose", 
            getRobotPose(new Pose2d())
        );
        Logger.getInstance().recordOutput(
            "Relative Tag Pose",
            getRelativeTagPose(new Pose2d())
        );
    }

    @Override
    public void fromLog(LogTable table) {}
}
