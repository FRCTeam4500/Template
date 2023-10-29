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
	private static Vision instanceVision;
	private Limelight hehehe;
	private Limelight haha;

	private Vision() {
		hehehe = new Limelight("limelight-hehehe");
		haha = new Limelight("limelight-haha");
	}

	public static synchronized Vision getInstance() {
		if (instanceVision == null) {
			instanceVision = new Vision();
		}
		return instanceVision;
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

	public Limelight getLimelight3() {
		return hehehe;
	}

	public Limelight getLimelight2() {
		return haha;
	}

	/** Gets the id of the april tag currently being targeted
	 * <p><strong>Note: Defaults to -1 </strong> */
	public int getTagId() {
		return hehehe.getTargetTagId();
	}

	/** Gets the robot's position relative to the current alliance <p>
	 * <strong>Note: Defaults to a pose at (0,0) with no rotation if no tag is seen</strong> */
	public Pose2d getRobotPose() {
		return getRobotPose(DriverStation.getAlliance());
	}

	/** Gets the robot's position relative to a specified alliance <p>
	 * <strong>Note: Defaults to a pose at (0,0) with no rotation if no tag is seen</strong>
	 * @param poseOrigin Sets which alliance the position is relative to */
	public Pose2d getRobotPose(Alliance poseOrigin) {
		return hehehe.getRobotPoseToAlliance(poseOrigin);
	}

	/** Gets the position of the currently targeted april tag relative to the robot 
	 * <p><strong>Note: Defaults to a pose at (0,0) with no rotation if no tag is seen</strong> */
	public Pose2d getRelativeTargetPose() {
		return hehehe.getTargetPoseToRobot();
	}

	/** Gets the horizontal angle of the limelight 2's target 
	 * <p><strong>Note: Defaults to 0 <strong> */
	public double getHorizontalAngleOffset() {
		return haha.getHorizontalOffsetFromCrosshair();
	}

	/** Gets the vertical angle of the limelight 2's target 
	 * <p><strong>Note: Defaults to 0 <strong> */
	public double getVerticalAngleOffset() {
		return haha.getVerticalOffsetFromCrosshair();
	}

	/** Gets the % area taken by the limelight 2's target 
	 * <p><strong>Note: Defaults to 0 <strong> */
	public double getTakenArea() {
		return haha.getTargetArea();
	}

	/** Gets the skew (rotation) of the limelight 2's target 
	 * <p><strong>Note: Defaults to 0 <strong> */
	public double getSkew() {
		return haha.getSkew();
	}

	/** Checks if the limelight 2 has any valid targets */
	public boolean hasValidTargets() {
		return haha.hasValidTargets();
	}

	/** Sets the vision pipeline of the limelight 2 */
	public void setPipeline(int pipeline) {
		haha.setPipeline(pipeline);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty("Haha: Valid Targets", () -> hasValidTargets(), null);
		builder.addDoubleProperty("Haha: Horizontal Offset (Degrees)", () -> Units.radiansToDegrees(getHorizontalAngleOffset()), null);
		builder.addDoubleProperty("Haha: Target Area (%)", () -> getTakenArea(), null);
	}
}