package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.CANConstants;
import frc.robot.hardware.EncodedMotorController;
import frc.robot.hardware.TalonFXMotorController;

public class SwerveConstants {
    // TODO: Change these!
    /** Drive rotations per motor rotation */
    public static final double DRIVE_RATIO = 1/5.;
    /** Angle rotations per motor rotation */
    public static final double ANGLE_RATIO = 1/6.75;

    public static final double MAX_LINEAR_SPEED_MPS = 5.088;
    public static final double WHEEL_DIAMETER_METERS = 0.0762;

    /* Sensitivities */
    public static final double MAX_FORWARD_SENSITIVITY = 4;
    public static final double MAX_SIDEWAYS_SENSITIVITY = 4;
    public static final double MAX_ROTATIONAL_SENSITIVITY = 3.5;

    public static final EncodedMotorController FRONT_LEFT_DRIVE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_FRONT_LEFT_DRIVE_ID)
            .setInversion(true)
            .setCurrentLimit(35)
            .setPID(new PIDConstants(0.075, 0, 0));
    public static final EncodedMotorController FRONT_LEFT_ANGLE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_FRONT_LEFT_ANGLE_ID)
            .setInversion(false)
            .setCurrentLimit(25)
            .setPID(new PIDConstants(0.3, 0, 0));
    public static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(
        0.3175,
        0.2413
    );

    public static final EncodedMotorController FRONT_RIGHT_DRIVE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_FRONT_RIGHT_DRIVE_ID)
            .setInversion(false)
            .setCurrentLimit(35)
            .setPID(new PIDConstants(0.05, 0, 0));
    public static final EncodedMotorController FRONT_RIGHT_ANGLE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_FRONT_RIGHT_ANGLE_ID)
            .setInversion(false)
            .setCurrentLimit(25)
            .setPID(new PIDConstants(0.3, 0, 0));
    public static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(
        0.3175,
        -0.2413
    );

    public static final EncodedMotorController BACK_LEFT_DRIVE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_BACK_LEFT_DRIVE_ID)
            .setInversion(true)
            .setCurrentLimit(35)
            .setPID(new PIDConstants(0.075, 0, 0));
    public static final EncodedMotorController BACK_LEFT_ANGLE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_BACK_LEFT_ANGLE_ID)
            .setInversion(false)
            .setCurrentLimit(25)
            .setPID(new PIDConstants(0.25, 0, 0));
    public static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new Translation2d(
        -0.3175,
        0.2431
    );

    public static final EncodedMotorController BACK_RIGHT_DRIVE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_BACK_RIGHT_DRIVE_ID)
            .setInversion(false)
            .setCurrentLimit(35)
            .setPID(new PIDConstants(0.05, 0, 0));
    public static final EncodedMotorController BACK_RIGHT_ANGLE_MOTOR = 
        new TalonFXMotorController(CANConstants.SWERVE_BACK_RIGHT_ANGLE_ID)
            .setInversion(false)
            .setCurrentLimit(25)
            .setPID(new PIDConstants(0.3, 0, 0));
    public static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new Translation2d(
        -0.3175,
        -0.2413
    );
}