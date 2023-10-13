package frc.robot.utilities;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import java.util.TimerTask;

import org.littletonrobotics.junction.Logger;

public class LogSubsystemInputsTask extends TimerTask {
	SwerveDrive swerve;
	MessagingSystem messagingSystem;
	Vision vision;
	private Loggable[] thingsToLog = {
		SwerveDrive.getInstance(),
		Vision.getInstance(),
		MessagingSystem.getInstance()
	};
	private LogInputs inputHandler;


	public LogSubsystemInputsTask() {
		this.swerve = SwerveDrive.getInstance();
		this.messagingSystem = MessagingSystem.getInstance();
		this.vision = Vision.getInstance();
		inputHandler = LogInputs.getInstance();
	}

	@Override
	public void run() {
		for (int i = 0; i < thingsToLog.length; i++) {
			inputHandler.setLoggingTarget(thingsToLog[i]);
			Logger.getInstance().processInputs(thingsToLog[i].getTableName(), inputHandler);
		}
		Logger.getInstance().recordOutput("Odometry", swerve.getRobotPose());
		Logger.getInstance().recordOutput("Vision Odometry", vision.getLimelight(0).getRobotPoseToAlliance(Alliance.Red));
		SwerveModuleState[] states = swerve.getModuleStates();
		Logger
			.getInstance()
			.recordOutput(
				"ModuleStates",
				states[0],
				states[1],
				states[2],
				states[3]
			);
	}
}
