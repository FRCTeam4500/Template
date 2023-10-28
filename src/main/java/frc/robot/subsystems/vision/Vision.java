package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Limelight;
import frc.robot.utilities.Loggable;

public class Vision extends SubsystemBase implements Loggable{
	private static Vision instance;
	private Limelight aprilTagLimelight;
	private Limelight gamePieceLimelight;

	private Vision() {
		aprilTagLimelight = new Limelight("limelight-hehehe");
		gamePieceLimelight = new Limelight("limelight-haha");
	}

	public static synchronized Vision getInstance() {
		return instance == null ? new Vision() : instance;
	}

	@Override
	public void logData(Logger logger, LogTable table) {
		table.put("Tag ID", getTagId());
		logger.recordOutput("Vision Odometry", getRobotPose());
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

	/** Defaults to -1 */
	public int getTagId() {
		return aprilTagLimelight.getTargetTagId();
	}

	/** Defaults to a default Pose2d */
	public Pose2d getRobotPose() {
		return getRobotPose(DriverStation.getAlliance());
	}

	/** Defaults to a default Pose2d */
	public Pose2d getRobotPose(Alliance poseOrigin) {
		return aprilTagLimelight.getRobotPoseToAlliance(poseOrigin);
	}

	/** Defaults to a default Pose2d */
	public Pose2d getRelativeTargetPose() {
		return aprilTagLimelight.getTargetPoseToRobot();
	}

	/** Defaults to 0 */
	public double getGamePieceHorizontalAngleOffset() {
		return gamePieceLimelight.getHorizontalOffsetFromCrosshair();
	}

	/** Defaults to 0 */
	public double getGamePieceVerticalAngleOffset() {
		return gamePieceLimelight.getVerticalOffsetFromCrosshair();
	}

	/** Defaults to 0 */
	public double getGamePieceTakenArea() {
		return gamePieceLimelight.getTargetArea();
	}

	/** Defaults to 0 */
	public double getGamePieceSkew() {
		return gamePieceLimelight.getSkew();
	}

	public boolean seesGamePieces() {
		return gamePieceLimelight.hasValidTargets();
	}
	
	public void setGamePiecePipeline(int pipeline) {
		gamePieceLimelight.setPipeline(pipeline);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty("Game Piece Limelight: Valid Targets", () -> seesGamePieces(), null);
		builder.addDoubleProperty("Game Piece Limelight: Horizontal Offset (Degrees)", () -> Units.radiansToDegrees(getGamePieceHorizontalAngleOffset()), null);
		builder.addDoubleProperty("Game Piece Limelight: Target Area (%)", () -> getGamePieceTakenArea(), null);
	}
}