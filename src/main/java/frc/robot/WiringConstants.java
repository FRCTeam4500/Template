package frc.robot;

/**
 * A class that holds wiring details for the robot. These include CAN IDs for motors, as well as
 * ports for the RIO. Each subsystem has its own subclass with subsystem specific information
 */
public class WiringConstants {
  /**
   * A class containing the wiring details for the swerve subsystem. Holds the 8 CAN IDs, as well as
   * DIO ports for the absolute encoders
   */
  public static class SwerveWiring {
    public static final int FRONT_LEFT_DRIVE_ID = 22;
    public static final int FRONT_LEFT_ANGLE_ID = 13;
    public static final int FRONT_RIGHT_DRIVE_ID = 6;
    public static final int FRONT_RIGHT_ANGLE_ID = 3;
    public static final int BACK_LEFT_DRIVE_ID = 21;
    public static final int BACK_LEFT_ANGLE_ID = 24;
    public static final int BACK_RIGHT_DRIVE_ID = 5;
    public static final int BACK_RIGHT_ANGLE_ID = 7;

    public static final int FRONT_LEFT_ENCODER_ID = 0;
    public static final int FRONT_RIGHT_ENCODER_ID = 1;
    public static final int BACK_LEFT_ENCODER_ID = 3;
    public static final int BACK_RIGHT_ENCODER_ID = 2;
  }
}
