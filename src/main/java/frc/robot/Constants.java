package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveModule.SwerveMotorConfig;

public class Constants {
	public static class JoystickConstants {
		public static final int DRIVER_PORT = 2;
	}

	public static class SwerveConstants {
		
		public static final double MAX_LINEAR_SPEED = 0;
		public static final double MAX_LINEAR_ACCELERATION = 0; 
		public static final double MAX_ROTATIONAL_SPEED = 0.0;
		public static final double MAX_ROTATIONAL_ACCELERATION = 0.0;

		public static final double DRIVE_RATIO = 1 / 1.; // drive rotations per motor rotation
		public static final double ANGLE_RATIO = 1 / 1.; // angle rotations per motor rotation

		public static final SwerveMotorConfig FRONT_LEFT_DRIVE_CONFIG = new SwerveMotorConfig(
			0, 
			true, 
			true, 
			false, 
			35, 
			new PIDConstants(0.075, 0, 0)
		);
		public static final SwerveMotorConfig FRONT_LEFT_ANGLE_CONFIG = new SwerveMotorConfig(
			0, 
			false, 
			false, 
			false, 25, 
			new PIDConstants(0.3, 0, 0)
		);
		public static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(
			0,
			0
		);

		public static final SwerveMotorConfig FRONT_RIGHT_DRIVE_CONFIG = new SwerveMotorConfig(
			0, 
			true, 
			false, 
			false, 
			35, 
			new PIDConstants(0.05, 0, 0)
		);
		public static final SwerveMotorConfig FRONT_RIGHT_ANGLE_CONFIG = new SwerveMotorConfig(
			0, 
			false, 
			false, 
			false, 
			25, 
			new PIDConstants(0.3, 0, 0)
		);
		public static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(
			0,
			0
		);

		public static final SwerveMotorConfig BACK_LEFT_DRIVE_CONFIG = new SwerveMotorConfig(
			0, 
			true, 
			true, 
			false, 
			35, 
			new PIDConstants(0.075, 0, 0)
		);
		public static final SwerveMotorConfig BACK_LEFT_ANGLE_CONFIG = new SwerveMotorConfig(
			0, 
			false, 
			false, 
			false, 25, 
			new PIDConstants(0.25, 0, 0)
		);
		public static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new Translation2d(
			0,
			0
		);

		public static final SwerveMotorConfig BACK_RIGHT_DRIVE_CONFIG = new SwerveMotorConfig(
			0, 
			true, 
			false, 
			false, 
			35, 
			new PIDConstants(0.05, 0, 0)
		);
		public static final SwerveMotorConfig BACK_RIGHT_ANGLE_CONFIG = new SwerveMotorConfig(
			0, 
			false, 
			false, 
			false, 
			25, 
			new PIDConstants(0.3, 0, 0)
		);
		public static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new Translation2d(
			0, 
			0
		);
		
		public static final double WHEEL_DIAMETER = 0.0;
	}

	public static class VisionConstants {
		public static int APRIL_TAG_PIPLINE = 0;
		public static int REFLECTIVE_TAPE_PIPELINE = 0;
	}

	public static class EnumConstants {
		public static enum DriveMode {
			AngleCentric,
			RobotCentric
		}

		public static enum CameraMode {
			VisionProcessor,
			DriverCamera,
		}

		public static enum TalonModel {
			TalonFX("Talon FX", 2048 / Math.PI / 2),
			TalonSRX("Talon SRX", 4096 / Math.PI / 2);
			
			public String name;
			public double ticksPerRadian;
			private TalonModel(String name, double ticksPerRadian) {
				this.name = name;
				this.ticksPerRadian = ticksPerRadian;
			}
		}
	}
}
