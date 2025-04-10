package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.subsystems.orchestra.Orc;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.logging.HoundLog;

@SuppressWarnings("resource")
public class SwerveConstants {

  /** The max speed the robot should travel at */
  public static final ChassisSpeeds MAX_FIELD_REL_SPEEDS = new ChassisSpeeds(3.75, 3.75, 6);

  public static final ChassisSpeeds MAX_ROBOT_REL_SPEEDS = new ChassisSpeeds(5, 3, 6);

  /** The minimum coefficient for slowmode */
  public static final double MIN_COEFFICIENT = 0.14546;

  /** The absolute max acheivable module speed */
  public static final double MAX_MODULE_SPEED = 5.4;

  /** A coefficient used to correct from translation while rotating */
  public static final double SKEW_COEFFICIENT = -0.129;

  /** The position of the front left module from the robot's center */
  public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(0.368, 0.266);

  /** The position of the front right module from the robot's center */
  public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(0.368, -0.266);

  /** The position of the back left module from the robot's center */
  public static final Translation2d BACK_LEFT_TRANSLATION = new Translation2d(-0.368, 0.266);

  /** The position of the back right module from the robot's center */
  public static final Translation2d BACK_RIGHT_TRANSLATION = new Translation2d(-0.368, -0.266);

  public static final SwerveModule FRONT_LEFT_MODULE =
      new SwerveModule(
          Motor.fromTalonFX(
              11,
              motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(60)
                        .withSupplyCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(17.5);
                StatusCode status = StatusCode.StatusCodeNotInitialized;
                for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                  status = motor.getConfigurator().apply(config);
                }
                if (status != StatusCode.OK) {
                  HoundLog.logFault(
                      "[Swerve] Front Left Drive Motor Config Error: " + status.getName(),
                      AlertType.kError);
                } else {
                  Orc.addMotor(motor);
                }
              },
              sim -> {},
              0,
              FeedbackController.fromPID(0.1, 0, 0, controller -> {}),
              FeedforwardController.forConstantGravity(0, 0.19635, 2.0292, 0.19562),
              TargetType.Velocity),
          Motor.fromSparkMax(
              9,
              false,
              motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kCoast);
                config
                    .encoder
                    .positionConversionFactor(1.0 / 25 * 360)
                    .velocityConversionFactor(1.0 / 25 * 360);
                REVLibError err =
                    motor.configure(
                        config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                if (!err.equals(REVLibError.kOk)) {
                  HoundLog.logFault(
                      "[Swerve] Front Left Angle Motor Config Error: " + err.name(),
                      AlertType.kError);
                }
              },
              sim -> {},
              (new AnalogEncoder(0).get() - 0.642) * 360,
              FeedbackController.fromPID(
                  new PIDController(0.1, 0, 0),
                  controller -> {
                    controller.enableContinuousInput(0, 360);
                    controller.setTolerance(1);
                  }),
              FeedforwardController.forConstantGravity(0, 0.15603, 0.0085738, 0.0010808),
              TargetType.Position));

  public static final SwerveModule FRONT_RIGHT_MODULE =
      new SwerveModule(
          Motor.fromTalonFX(
              10,
              motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(60)
                        .withSupplyCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(17.5);
                StatusCode status = StatusCode.StatusCodeNotInitialized;
                for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                  status = motor.getConfigurator().apply(config);
                }
                if (status != StatusCode.OK) {
                  HoundLog.logFault(
                      "[Swerve] Front Right Drive Motor Config Error: " + status.getName(),
                      AlertType.kError);
                } else {
                  Orc.addMotor(motor);
                }
              },
              sim -> {},
              0,
              FeedbackController.fromPID(0.1, 0, 0, controller -> {}),
              FeedforwardController.forConstantGravity(0, 0.20427, 2.0144, 0.25467),
              TargetType.Velocity),
          Motor.fromSparkMax(
              8,
              false,
              motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kCoast);
                config
                    .encoder
                    .positionConversionFactor(1.0 / 25 * 360)
                    .velocityConversionFactor(1.0 / 25 * 360);
                REVLibError err =
                    motor.configure(
                        config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                if (!err.equals(REVLibError.kOk)) {
                  HoundLog.logFault(
                      "[Swerve] Front Right Angle Motor Config Error: " + err.name(),
                      AlertType.kError);
                }
              },
              sim -> {},
              (new AnalogEncoder(1).get() - 0.668) * 360,
              FeedbackController.fromPID(
                  new PIDController(0.1, 0, 0),
                  controller -> {
                    controller.enableContinuousInput(0, 360);
                    controller.setTolerance(1);
                  }),
              FeedforwardController.forConstantGravity(0, 0.27701, 0.0089885, 0.0010955),
              TargetType.Position));

  public static final SwerveModule BACK_LEFT_MODULE =
      new SwerveModule(
          Motor.fromTalonFX(
              19,
              motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(60)
                        .withSupplyCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(17.5);
                StatusCode status = StatusCode.StatusCodeNotInitialized;
                for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                  status = motor.getConfigurator().apply(config);
                }
                if (status != StatusCode.OK) {
                  HoundLog.logFault(
                      "[Swerve] Back Left Drive Motor Config Error: " + status.getName(),
                      AlertType.kError);
                } else {
                  Orc.addMotor(motor);
                }
              },
              sim -> {},
              0,
              FeedbackController.fromPID(0.1, 0, 0, controller -> {}),
              FeedforwardController.forConstantGravity(0, 0.2049, 2.0169, 0.2644),
              TargetType.Velocity),
          Motor.fromSparkMax(
              18,
              false,
              motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kCoast);
                config
                    .encoder
                    .positionConversionFactor(1.0 / 25 * 360)
                    .velocityConversionFactor(1.0 / 25 * 360);
                REVLibError err =
                    motor.configure(
                        config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                if (!err.equals(REVLibError.kOk)) {
                  HoundLog.logFault(
                      "[Swerve] Back Left Angle Motor Config Error: " + err.name(),
                      AlertType.kError);
                }
              },
              sim -> {},
              (new AnalogEncoder(2).get() - 0.022) * 360,
              FeedbackController.fromPID(
                  new PIDController(0.1, 0, 0),
                  controller -> {
                    controller.enableContinuousInput(0, 360);
                    controller.setTolerance(1);
                  }),
              FeedforwardController.forConstantGravity(0, 0.25886, 0.0090872, 0.0012662),
              TargetType.Position));

  public static final SwerveModule BACK_RIGHT_MODULE =
      new SwerveModule(
          Motor.fromTalonFX(
              7,
              motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(60)
                        .withSupplyCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(17.5);
                StatusCode status = StatusCode.StatusCodeNotInitialized;
                for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                  status = motor.getConfigurator().apply(config);
                }
                if (status != StatusCode.OK) {
                  HoundLog.logFault(
                      "[Swerve] Back Right Drive Motor Config Error: " + status.getName(),
                      AlertType.kError);
                } else {
                  Orc.addMotor(motor);
                }
              },
              sim -> {},
              0,
              FeedbackController.fromPID(0.1, 0, 0, controller -> {}),
              FeedforwardController.forConstantGravity(0, 0.20206, 2.0934, 0.18192),
              TargetType.Velocity),
          Motor.fromSparkMax(
              6,
              false,
              motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kCoast);
                config
                    .encoder
                    .positionConversionFactor(1.0 / 25 * 360)
                    .velocityConversionFactor(1.0 / 25 * 360);
                REVLibError err =
                    motor.configure(
                        config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                if (!err.equals(REVLibError.kOk)) {
                  HoundLog.logFault(
                      "[Swerve] Back Right Angle Motor Config Error: " + err.name(),
                      AlertType.kError);
                }
              },
              sim -> {},
              (new AnalogEncoder(3).get() - 0.879) * 360,
              FeedbackController.fromPID(
                  new PIDController(0.1, 0, 0),
                  controller -> {
                    controller.enableContinuousInput(0, 360);
                    controller.setTolerance(1);
                  }),
              FeedforwardController.forConstantGravity(0, 0.25348, 0.0092287, 0.0014289),
              TargetType.Position));

  public static class TagPoseCameraOffsets {
    public static final Transform2d limelightHeHeHe =
        new Transform2d(-0.431, -0.03, Rotation2d.fromRadians(0));
    public static final Transform2d limelightHiHiHi =
        new Transform2d(-0.431, 0.03, Rotation2d.fromRadians(0));
  }
}
