package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.base.SwerveModule.SwerveModuleConfig;
import frc.robot.subsystems.swerve.base.SwerveMotor;

import static frc.robot.CANConstants.*;
import static com.ctre.phoenix6.signals.InvertedValue.*;
import static com.revrobotics.CANSparkLowLevel.MotorType.*;

public class SwerveConstants {
    public static final double MAX_FORWARD_SPEED = 4;
    public static final double MAX_SIDEWAYS_SPEED = 4;
    public static final double MAX_ROTATIONAL_SPEED = 4;
    public static final double MIN_COEFFICIENT = 0.2;
    public static final double MAX_MODULE_SPEED = 4;
    public static final PathConstraints TELEOP_CONSTRAINTS = new PathConstraints(
        4, 3, 9.425, 12.5664
    );
    
    public static final TalonFXConfiguration driveConfig =
        new TalonFXConfiguration()
            .withSlot1(new Slot1Configs()
                .withKP(0.11)
                .withKI(0.5)
                .withKD(0.0001)
                .withKV(0.12))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(35)
                .withSupplyCurrentLimitEnable(true)
            );
    public static final SwerveMotor FRONT_LEFT_DRIVE_MOTOR =
        SwerveMotor.fromTalonFX(
            new TalonFX(SWERVE_FRONT_LEFT_DRIVE_ID),
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        );
    public static final SwerveMotor FRONT_LEFT_ANGLE_MOTOR =
        SwerveMotor.fromSparkMax(
            new CANSparkMax(SWERVE_FRONT_LEFT_ANGLE_ID, kBrushless),
            motor -> motor.getPIDController().setP(0.7)
        );
    public static final SwerveModuleConfig FRONT_LEFT_MODULE_CONFIG = new SwerveModuleConfig(
        new Translation2d(0.2974, 0.2974), 0.1016, 1/7.5, 1/6.75
    );

    public static final SwerveMotor FRONT_RIGHT_DRIVE_MOTOR =
        SwerveMotor.fromTalonFX(
            new TalonFX(SWERVE_FRONT_RIGHT_DRIVE_ID),
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        );
    public static final SwerveMotor FRONT_RIGHT_ANGLE_MOTOR =
        SwerveMotor.fromSparkMax(
            new CANSparkMax(SWERVE_FRONT_RIGHT_ANGLE_ID, kBrushless),
            motor -> motor.getPIDController().setP(0.8)
        );
    public static final SwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(
        new Translation2d(0.2974, -0.2974), 0.1016, 1/7.5, 1/6.75
    );

    public static final SwerveMotor BACK_LEFT_DRIVE_MOTOR =
        SwerveMotor.fromTalonFX(
            new TalonFX(SWERVE_BACK_LEFT_DRIVE_ID),
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        );
    public static final SwerveMotor BACK_LEFT_ANGLE_MOTOR =
        SwerveMotor.fromSparkMax(
            new CANSparkMax(SWERVE_BACK_LEFT_ANGLE_ID, kBrushless),
            motor -> motor.getPIDController().setP(0.75)
        );
    public static final SwerveModuleConfig BACK_LEFT_MODULE_CONFIG = new SwerveModuleConfig(
        new Translation2d(-0.2974, 0.2974), 0.1016, 1/7.5, 1/6.75
    );

    public static final SwerveMotor BACK_RIGHT_DRIVE_MOTOR =
        SwerveMotor.fromTalonFX(
            new TalonFX(SWERVE_BACK_RIGHT_DRIVE_ID),
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        );
    public static final SwerveMotor BACK_RIGHT_ANGLE_MOTOR =
        SwerveMotor.fromSparkMax(
            new CANSparkMax(SWERVE_BACK_RIGHT_ANGLE_ID, kBrushless),
            motor -> motor.getPIDController().setP(0.8)
        );
    public static final SwerveModuleConfig BACK_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(
        new Translation2d(-0.2974, -0.2974), 0.1016, 1/7.5, 1/6.75
    );
}
