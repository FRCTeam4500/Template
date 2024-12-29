package frc.robot.subsystems.swerve;

import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.*;
import static frc.robot.WiringConstants.SwerveWiring.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.FeedforwardConstants;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import java.util.Optional;

public class SwerveConstants {
  /** The max speed the robot should travel at */
  public static final ChassisSpeeds MAX_SPEEDS = new ChassisSpeeds(6, 6, 4);

  /** The minimum coefficient for slowmode */
  public static final double MIN_COEFFICIENT = 0.2;

  /** The absolute max acheivable module speed */
  public static final double MAX_MODULE_SPEED = 6;

  /** A coefficient used to correct from translation while rotating */
  public static final double SKEW_COEFFICIENT = -0.129;

  /** The position of the front left module from the robot's center */
  public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(0.2974, 0.2974);

  /** The position of the front right module from the robot's center */
  public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(0.2974, -0.2974);

  /** The position of the back left module from the robot's center */
  public static final Translation2d BACK_LEFT_TRANSLATION = new Translation2d(-0.2974, 0.2974);

  /** The position of the back right module from the robot's center */
  public static final Translation2d BACK_RIGHT_TRANSLATION = new Translation2d(-0.2974, -0.2974);

  public static final SwerveModule FRONT_LEFT_MODULE =
      new SwerveModule(
          Motor.fromTalonFX(
              FRONT_LEFT_DRIVE_ID,
              motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(12.1908);
                StatusCode status = StatusCode.StatusCodeNotInitialized;
                for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                  status = motor.getConfigurator().apply(config);
                }
              },
              sim -> {},
              0,
              FeedbackController.fromPID(new PIDController(0.004, 0, 0), controller -> {}),
              // Stolen from front right, sysID kA gain was wierd
              Optional.of(new FeedforwardConstants(0, 0.10624, 1.407, 0.16994)),
              TargetType.Velocity),
          Motor.fromSparkMax(
              FRONT_LEFT_ANGLE_ID,
              false,
              motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
                config
                    .encoder
                    .positionConversionFactor(1.0 / 25)
                    .velocityConversionFactor(1.0 / 25 / 60);
                motor.configure(config, kResetSafeParameters, kPersistParameters);
                AnalogEncoder absoluteEncoder = new AnalogEncoder(FRONT_LEFT_ENCODER_ID);
                motor.getEncoder().setPosition(absoluteEncoder.get() - 0.805);
                absoluteEncoder.close();
              },
              sim -> {},
              0,
              FeedbackController.fromPID(
                  new PIDController(27, 0, 0),
                  controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                  }),
              Optional.of(new FeedforwardConstants(0, 0.18487, 2.953, 0.22385)),
              TargetType.Position));

  public static final SwerveModule FRONT_RIGHT_MODULE =
      new SwerveModule(
          Motor.fromTalonFX(
              FRONT_RIGHT_DRIVE_ID,
              motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(12.1908);
                StatusCode status = StatusCode.StatusCodeNotInitialized;
                for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                  status = motor.getConfigurator().apply(config);
                }
              },
              sim -> {},
              0,
              FeedbackController.fromPID(new PIDController(0.004, 0, 0), controller -> {}),
              Optional.of(new FeedforwardConstants(0, 0.10624, 1.407, 0.16994)),
              TargetType.Velocity),
          Motor.fromSparkMax(
              FRONT_RIGHT_ANGLE_ID,
              false,
              motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
                config
                    .encoder
                    .positionConversionFactor(1.0 / 25)
                    .velocityConversionFactor(1.0 / 25 / 60);
                motor.configure(config, kResetSafeParameters, kPersistParameters);
                AnalogEncoder absoluteEncoder = new AnalogEncoder(FRONT_RIGHT_ENCODER_ID);
                motor.getEncoder().setPosition(absoluteEncoder.get() - 0.394);
                absoluteEncoder.close();
              },
              sim -> {},
              0,
              FeedbackController.fromPID(
                  new PIDController(27, 0, 0),
                  controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                  }),
              Optional.of(new FeedforwardConstants(0, 0.18487, 2.953, 0.22385)),
              TargetType.Position));

  public static final SwerveModule BACK_LEFT_MODULE =
      new SwerveModule(
          Motor.fromTalonFX(
              BACK_LEFT_DRIVE_ID,
              motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(12.1908);
                StatusCode status = StatusCode.StatusCodeNotInitialized;
                for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                  status = motor.getConfigurator().apply(config);
                }
              },
              sim -> {},
              0,
              FeedbackController.fromPID(new PIDController(0.004, 0, 0), controller -> {}),
              Optional.of(new FeedforwardConstants(0, 0.11833, 1.3984, 0.16306)),
              TargetType.Velocity),
          Motor.fromSparkMax(
              BACK_LEFT_ANGLE_ID,
              false,
              motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
                config
                    .encoder
                    .positionConversionFactor(1.0 / 25)
                    .velocityConversionFactor(1.0 / 25 / 60);
                motor.configure(config, kResetSafeParameters, kPersistParameters);
                AnalogEncoder absoluteEncoder = new AnalogEncoder(BACK_LEFT_ENCODER_ID);
                motor.getEncoder().setPosition(absoluteEncoder.get() - 0.511);
                absoluteEncoder.close();
              },
              sim -> {},
              0,
              FeedbackController.fromPID(
                  new PIDController(27, 0, 0),
                  controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                  }),
              Optional.of(new FeedforwardConstants(0, 0.18487, 2.953, 0.22385)),
              TargetType.Position));

  public static final SwerveModule BACK_RIGHT_MODULE =
      new SwerveModule(
          Motor.fromTalonFX(
              BACK_RIGHT_DRIVE_ID,
              motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(12.1908);
                StatusCode status = StatusCode.StatusCodeNotInitialized;
                for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                  status = motor.getConfigurator().apply(config);
                }
              },
              sim -> {},
              0,
              FeedbackController.fromPID(new PIDController(0.004, 0, 0), controller -> {}),
              Optional.of(new FeedforwardConstants(0, 0.10512, 1.3954, 0.19264)),
              TargetType.Velocity),
          Motor.fromSparkMax(
              BACK_RIGHT_ANGLE_ID,
              false,
              motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
                config
                    .encoder
                    .positionConversionFactor(1.0 / 25)
                    .velocityConversionFactor(1.0 / 25 / 60);
                motor.configure(config, kResetSafeParameters, kPersistParameters);
                AnalogEncoder absoluteEncoder = new AnalogEncoder(BACK_RIGHT_ENCODER_ID);
                motor.getEncoder().setPosition(absoluteEncoder.get() - 0.061);
                absoluteEncoder.close();
              },
              sim -> {},
              0,
              FeedbackController.fromPID(
                  new PIDController(27, 0, 0),
                  controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                  }),
              Optional.of(new FeedforwardConstants(0, 0.18487, 2.953, 0.22385)),
              TargetType.Position));
}
