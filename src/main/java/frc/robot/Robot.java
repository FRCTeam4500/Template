package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utilities.LogSubsystemInputsTask;

import java.util.Timer;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
	private RobotContainer robotContainer;
	private Timer timer;

	@Override
	public void robotInit() {
		Logger logger = Logger.getInstance();
		timer = new Timer();

		logger.recordMetadata("ProjectName", BuildInfo.MAVEN_NAME);
		logger.recordMetadata("BuildDate", BuildInfo.BUILD_DATE);
		logger.recordMetadata("GitSHA", BuildInfo.GIT_SHA);
		logger.recordMetadata("GitDate", BuildInfo.GIT_DATE);
		logger.recordMetadata("GitBranch", BuildInfo.GIT_BRANCH);
		switch (BuildInfo.DIRTY) {
			case 0:
				logger.recordMetadata("GitDirty", "All changes committed");
				break;
			case 1:
				logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;
			default:
				logger.recordMetadata("GitDirty", "Unknown");
				break;
		}
		logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
		logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
		LoggedPowerDistribution.getInstance(1, ModuleType.kRev); // Enables power distribution logging

		logger.start();
		robotContainer = new RobotContainer();
		timer.schedule(new LogSubsystemInputsTask(), 10, 20);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		robotContainer.autonomousInit();
	}
	
	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		robotContainer.teleopInit();
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void testInit() {}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
