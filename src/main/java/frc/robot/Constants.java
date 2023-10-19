package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveModule.SwerveMotorConfig;

public class Constants {
	public static class JoystickConstants {
		// Joystick ports
		public static final int DRIVER_PORT = 2;
		public static final int OPERATOR_PORT = 1;
	}

	public static class SwerveConstants {
		
		public static final double MAX_LINEAR_SPEED =
			((1276 * 9.42) / 60) / 12 * 0.3048; // 1276 is rpm, 9.42 is wheel circumference (in.), final units are m/s
		public static final double MAX_LINEAR_ACCELERATION = 4; //Test
		public static final double MAX_ROTATIONAL_SPEED =
			MAX_LINEAR_SPEED / (4 / 3); // 4/3 is (about) the radius from the center of the robot to the swerve drive wheels.
		public static final double MAX_ROTATIONAL_ACCELERATION = 4; // Linear Acceleration/radius

		public static final double DRIVE_RATIO = 1 / 5.; // drive rotations per motor rotation
		public static final double ANGLE_RATIO = 1 / 6.75; // angle rotations per motor rotation

		public static final SwerveMotorConfig FRONT_LEFT_DRIVE_CONFIG = new SwerveMotorConfig(
			3, 
			true, 
			true, 
			false, 
			35, 
			new PIDConstants(0.075, 0, 0)
		);
		public static final SwerveMotorConfig FRONT_LEFT_ANGLE_CONFIG = new SwerveMotorConfig(
			7, 
			false, 
			false, 
			false, 25, 
			new PIDConstants(0.3, 0, 0)
		);
		public static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(
			0.3175,
			0.2413
		);

		public static final SwerveMotorConfig FRONT_RIGHT_DRIVE_CONFIG = new SwerveMotorConfig(
			4, 
			true, 
			false, 
			false, 
			35, 
			new PIDConstants(0.05, 0, 0)
		);
		public static final SwerveMotorConfig FRONT_RIGHT_ANGLE_CONFIG = new SwerveMotorConfig(
			8, 
			false, 
			false, 
			false, 
			25, 
			new PIDConstants(0.3, 0, 0)
		);
		public static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(
			0.3175,
			-0.2413
		);

		public static final SwerveMotorConfig BACK_LEFT_DRIVE_CONFIG = new SwerveMotorConfig(
			2, 
			true, 
			true, 
			false, 
			35, 
			new PIDConstants(0.075, 0, 0)
		);
		public static final SwerveMotorConfig BACK_LEFT_ANGLE_CONFIG = new SwerveMotorConfig(
			7, 
			false, 
			false, 
			false, 25, 
			new PIDConstants(0.25, 0, 0)
		);
		public static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new Translation2d(
			-0.3175,
			0.2413
		);

		public static final SwerveMotorConfig BACK_RIGHT_DRIVE_CONFIG = new SwerveMotorConfig(
			9, 
			true, 
			false, 
			false, 
			35, 
			new PIDConstants(0.05, 0, 0)
		);
		public static final SwerveMotorConfig BACK_RIGHT_ANGLE_CONFIG = new SwerveMotorConfig(
			8, 
			false, 
			false, 
			false, 
			25, 
			new PIDConstants(0.3, 0, 0)
		);
		public static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new Translation2d(
			-0.3175,
			-0.2413
		);


		public static final int DBRPORT = 9; //drive back right port
		public static final int DBLPORT = 2; //drive back left port
		public static final int DFLPORT = 3; //drive front left port
		public static final int DFRPORT = 4; //drive front right port
		public static final int ABRPORT = 5; //angle back right port
		public static final int ABLPORT = 6; //angle back left port
		public static final int AFLPORT = 7; //angle front left port
		public static final int AFRPORT = 8; //angle front right port

		public static final double WHEEL_DIAMETER = 0.0762; // in meters


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

		public static enum TalonModel {
			TalonFX("Talon FX"),
			TalonSRX("Talon SRX");
			public String model;
			private TalonModel(String model) {
				this.model = model;
			}
		}
	}
}
