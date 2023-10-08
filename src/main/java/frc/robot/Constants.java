package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
	public static class JoystickConstants {
		// Joystick ports
		public static final int DRIVER_PORT = 2;
		public static final int OPERATOR_PORT = 1;
	}

	public static class SwerveConstants {
		public static final double MAX_LINEAR_SPEED = 0;
		public static final double MAX_LINEAR_ACCELERATION = 0; 
		public static final double MAX_ROTATIONAL_SPEED = 0;
		public static final double MAX_ROTATIONAL_ACCELERATION = 0;

		public static final double DRIVE_RATIO = 0;
		public static final double ANGLE_RATIO = 0;

		public static final int DBRPORT = 0; //drive back right port
		public static final int DBLPORT = 0; //drive back left port
		public static final int DFLPORT = 0; //drive front left port
		public static final int DFRPORT = 0; //drive front right port
		public static final int ABRPORT = 0; //angle back right port
		public static final int ABLPORT = 0; //angle back left port
		public static final int AFLPORT = 0; //angle front left port
		public static final int AFRPORT = 0; //angle front right port

		public static final double WHEEL_DIAMETER = 0; // in meters

		// These are the translations of the swerve modules from the center of the robot.
		// Specifically, these measurments should land on the line that the swerve module wheel rotates around
		// Units are meters
		public static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(0, 0);
		public static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(0, 0);
		public static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new Translation2d(0, 0);
		public static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new Translation2d(0, 0);
	}
}
