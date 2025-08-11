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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.WiringConstants.SwerveWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.subsystems.orchestra.Orc;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedbackController.FeedbackConstants;
import frc.robot.utilities.logging.HoundLog;

@SuppressWarnings("resource")
public class SwerveConstants {

  /** The max speed the robot should travel at */
  public static final ChassisSpeeds MAX_TELEOP_SPEEDS = new ChassisSpeeds(3.75, 3.75, 6);

  /** The fastest speed the robot can theoretically travel at */
  public static final ChassisSpeeds MAX_ROBOT_SPEEDS = new ChassisSpeeds(5, 5, 6);

  /** The minimum coefficient for slowmode. */
  public static final double SLOWEST_COEFFICIENT = 0.14546;

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

  /** Config for the swerve modules... 
   * @param driveSupplyCurrentLimit The supply current limit for the drive motor.
   * @param driveSupplyCurrentLimitEnable Whether to enable supply current limiting on the drive motor
   * @param driveStatorCurrentLimit The stator current limit for the drive motor
   * @param driveStatorCurrentLimitEnable Whether to enable stator current limiting on the drive motor
   * @param driveConversionFactor Number that converts drive motor rotations to drive module meters traveled. Use the {@link Swerve#driveConversionFinder driveConversionFinder} command in Swerve.java to find these values!
   * @param drivePID PID values for the drive motor 
   * @param driveFeedforward The feedforward controller for the drive motor (SysID)
   * @param angleStatorCurrentLimit The stator current limit for the angle motor
   * @param angleGearReduction The gear ratio of the angle motor, either provided by cad team or the producer
   * @param angleAbsoluteEncoderOffset Reading of the absolute encoder when the module is faced forward (small gear on outside)
   * @param anglePID PID values for the drive motor
   * @param angleTolerance How close the module’s angle must be to the target to stop turning, in degrees)
   * @param angleFeedforward The feedforward controller for the angle motor (SysID)
   */
  public static record ModuleConfig(
    double driveSupplyCurrentLimit,
    boolean driveSupplyCurrentLimitEnable,
    double driveStatorCurrentLimit,
    boolean driveStatorCurrentLimitEnable,
    double driveConversionFactor,
    FeedbackConstants drivePID,
    FeedforwardController driveFeedforward,
    int angleStatorCurrentLimit,
    double angleGearReduction,
    double angleAbsoluteEncoderOffset,
    FeedbackConstants anglePID,
    double angleTolerance,
    FeedforwardController angleFeedforward
  ) {}

  /** Configuration for FRONT_LEFT_MODULE. see ModuleConfig to see what values correspond to. */
  public static final ModuleConfig FRONT_LEFT_CONFIG = new ModuleConfig(
    60, 
    true, 
    0, 
    false, 
    17.5, 
    new FeedbackConstants(0.1, 0, 0), 
    FeedforwardController.forConstantGravity(0, 0.19635, 2.0292, 0.19562), 
    20, 
    25, 
    0.642, 
    new FeedbackConstants(0.1, 0, 0), 
    1, 
    FeedforwardController.forConstantGravity(0, 0.15603, 0.0085738, 0.0010808)
  );

  /** Configuration for FRONT_RIGHT_MODULE. see ModuleConfig to see what values correspond to. */
  public static final ModuleConfig FRONT_RIGHT_CONFIG = new ModuleConfig(
    60, 
    true, 
    0, 
    false, 
    17.5, 
    new FeedbackConstants(0.1, 0, 0), 
    FeedforwardController.forConstantGravity(0, 0.20427, 2.0144, 0.25467), 
    20, 
    25, 
    0.668, 
    new FeedbackConstants(0.1, 0, 0), 
    1, 
    FeedforwardController.forConstantGravity(0, 0.27701, 0.0089885, 0.0010955)
  );
  
  /** Configuration for BACK_LEFT_MODULE. see ModuleConfig to see what values correspond to. */
  public static final ModuleConfig BACK_LEFT_CONFIG = new ModuleConfig(
    60, 
    true, 
    0, 
    false, 
    17.5, 
    new FeedbackConstants(0.1, 0, 0), 
    FeedforwardController.forConstantGravity(0, 0.2049, 2.0169, 0.2644), 
    20, 
    25, 
    0.022, 
    new FeedbackConstants(0.1, 0, 0), 
    1, 
    FeedforwardController.forConstantGravity(0, 0.25886, 0.0090872, 0.0012662)
  );
  
  /** Configuration for BACK_RIGHT_MODULE. see ModuleConfig to see what values correspond to. */
  public static final ModuleConfig BACK_RIGHT_CONFIG = new ModuleConfig(
    60, 
    true, 
    0, 
    false, 
    17.5, 
    new FeedbackConstants(0.1, 0, 0), 
    FeedforwardController.forConstantGravity(0, 0.20206, 2.0934, 0.18192), 
    20, 
    25, 
    0.879, 
    new FeedbackConstants(0.1, 0, 0), 
    1, 
    FeedforwardController.forConstantGravity(0, 0.25348, 0.0092287, 0.0014289)
  );

  public static final SwerveModule FRONT_LEFT_MODULE =
      new SwerveModule(
          Motor.fromTalonFX(
              SwerveWiring.FRONT_LEFT_DRIVE_ID,
              motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(FRONT_LEFT_CONFIG.driveSupplyCurrentLimit)
                        .withSupplyCurrentLimitEnable(FRONT_LEFT_CONFIG.driveSupplyCurrentLimitEnable)
                        .withStatorCurrentLimit(FRONT_LEFT_CONFIG.driveStatorCurrentLimit)
                        .withStatorCurrentLimitEnable(FRONT_LEFT_CONFIG.driveStatorCurrentLimitEnable);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(FRONT_LEFT_CONFIG.driveConversionFactor);
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
              FeedbackController.fromPID(FRONT_LEFT_CONFIG.drivePID, controller -> {}),
              FRONT_LEFT_CONFIG.driveFeedforward,
              TargetType.Velocity),
          Motor.fromSparkMax(
              SwerveWiring.FRONT_LEFT_ANGLE_ID,
              false,
              motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(FRONT_LEFT_CONFIG.angleStatorCurrentLimit).idleMode(IdleMode.kCoast);
                config
                    .encoder
                    .positionConversionFactor(1.0 / FRONT_LEFT_CONFIG.angleGearReduction * 360)
                    .velocityConversionFactor(1.0 / FRONT_LEFT_CONFIG.angleGearReduction * 360);
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
              (new AnalogEncoder(SwerveWiring.FRONT_LEFT_ENCODER_ID).get() - FRONT_LEFT_CONFIG.angleAbsoluteEncoderOffset) * 360,
              FeedbackController.fromPID(
                  FRONT_LEFT_CONFIG.anglePID,
                  controller -> {
                    controller.enableContinuousInput(0, 360);
                    controller.setTolerance(FRONT_LEFT_CONFIG.angleTolerance);
                  }),
              FRONT_LEFT_CONFIG.angleFeedforward,
              TargetType.Position));


  public static final SwerveModule FRONT_RIGHT_MODULE =
      new SwerveModule(
          Motor.fromTalonFX(
              SwerveWiring.FRONT_RIGHT_DRIVE_ID,
              motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(FRONT_RIGHT_CONFIG.driveSupplyCurrentLimit)
                        .withSupplyCurrentLimitEnable(FRONT_RIGHT_CONFIG.driveSupplyCurrentLimitEnable)
                        .withStatorCurrentLimit(FRONT_RIGHT_CONFIG.driveStatorCurrentLimit)
                        .withSupplyCurrentLimitEnable(FRONT_RIGHT_CONFIG.driveStatorCurrentLimitEnable);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(FRONT_RIGHT_CONFIG.driveConversionFactor);
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
              FeedbackController.fromPID(FRONT_RIGHT_CONFIG.drivePID, controller -> {}),
              FRONT_RIGHT_CONFIG.driveFeedforward,
              TargetType.Velocity),
          Motor.fromSparkMax(
              SwerveWiring.FRONT_RIGHT_ANGLE_ID,
              false,
              motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(FRONT_RIGHT_CONFIG.angleStatorCurrentLimit).idleMode(IdleMode.kCoast);
                config
                    .encoder
                    .positionConversionFactor(1.0 / FRONT_RIGHT_CONFIG.angleGearReduction * 360)
                    .velocityConversionFactor(1.0 / FRONT_RIGHT_CONFIG.angleGearReduction * 360);
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
              (new AnalogEncoder(SwerveWiring.FRONT_RIGHT_ENCODER_ID).get() - FRONT_RIGHT_CONFIG.angleAbsoluteEncoderOffset) * 360,
              FeedbackController.fromPID(
                  FRONT_RIGHT_CONFIG.anglePID,
                  controller -> {
                    controller.enableContinuousInput(0, 360);
                    controller.setTolerance(FRONT_RIGHT_CONFIG.angleTolerance);
                  }),
              FRONT_RIGHT_CONFIG.angleFeedforward,
              TargetType.Position));


  public static final SwerveModule BACK_LEFT_MODULE =
      new SwerveModule(
          Motor.fromTalonFX(
              SwerveWiring.BACK_LEFT_DRIVE_ID,
              motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(BACK_LEFT_CONFIG.driveSupplyCurrentLimit)
                        .withSupplyCurrentLimitEnable(BACK_LEFT_CONFIG.driveSupplyCurrentLimitEnable)
                        .withStatorCurrentLimit(BACK_LEFT_CONFIG.driveStatorCurrentLimit)
                        .withStatorCurrentLimitEnable(BACK_LEFT_CONFIG.driveStatorCurrentLimitEnable);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(BACK_LEFT_CONFIG.driveConversionFactor);
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
              FeedbackController.fromPID(BACK_LEFT_CONFIG.drivePID, controller -> {}),
              BACK_LEFT_CONFIG.driveFeedforward,
              TargetType.Velocity),
          Motor.fromSparkMax(
              SwerveWiring.BACK_LEFT_ANGLE_ID,
              false,
              motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(BACK_LEFT_CONFIG.angleStatorCurrentLimit).idleMode(IdleMode.kCoast);
                config
                    .encoder
                    .positionConversionFactor(1.0 / BACK_LEFT_CONFIG.angleGearReduction * 360)
                    .velocityConversionFactor(1.0 / BACK_LEFT_CONFIG.angleGearReduction * 360);
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
              (new AnalogEncoder(SwerveWiring.BACK_LEFT_ENCODER_ID).get() - BACK_LEFT_CONFIG.angleAbsoluteEncoderOffset) * 360,
              FeedbackController.fromPID(
                  BACK_LEFT_CONFIG.anglePID,
                  controller -> {
                    controller.enableContinuousInput(0, 360);
                    controller.setTolerance(BACK_LEFT_CONFIG.angleTolerance);
                  }),
              BACK_LEFT_CONFIG.angleFeedforward,
              TargetType.Position));
  

  public static final SwerveModule BACK_RIGHT_MODULE =
  new SwerveModule(
      Motor.fromTalonFX(
          SwerveWiring.BACK_RIGHT_DRIVE_ID,
          motor -> {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits =
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(BACK_RIGHT_CONFIG.driveSupplyCurrentLimit)
                    .withSupplyCurrentLimitEnable(BACK_RIGHT_CONFIG.driveStatorCurrentLimitEnable)
                    .withStatorCurrentLimit(BACK_RIGHT_CONFIG.driveStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(BACK_RIGHT_CONFIG.driveStatorCurrentLimitEnable);
            config.MotorOutput =
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive);
            config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(BACK_RIGHT_CONFIG.driveConversionFactor);
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
          FeedbackController.fromPID(BACK_RIGHT_CONFIG.drivePID, controller -> {}),
          BACK_RIGHT_CONFIG.driveFeedforward,
          TargetType.Velocity),
      Motor.fromSparkMax(
          SwerveWiring.BACK_RIGHT_ANGLE_ID,
          false,
          motor -> {
            SparkMaxConfig config = new SparkMaxConfig();
            config.inverted(false).smartCurrentLimit(BACK_RIGHT_CONFIG.angleStatorCurrentLimit).idleMode(IdleMode.kCoast);
            config
                .encoder
                .positionConversionFactor(1.0 / BACK_RIGHT_CONFIG.angleGearReduction * 360)
                .velocityConversionFactor(1.0 / BACK_RIGHT_CONFIG.angleGearReduction * 360);
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
          (new AnalogEncoder(SwerveWiring.BACK_RIGHT_ENCODER_ID).get() - BACK_RIGHT_CONFIG.angleAbsoluteEncoderOffset) * 360,
          FeedbackController.fromPID(
              BACK_RIGHT_CONFIG.anglePID,
              controller -> {
                controller.enableContinuousInput(0, 360);
                controller.setTolerance(BACK_RIGHT_CONFIG.angleTolerance);
              }),
          BACK_RIGHT_CONFIG.angleFeedforward,
          TargetType.Position));

}
