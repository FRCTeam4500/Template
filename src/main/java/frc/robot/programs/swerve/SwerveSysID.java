package frc.robot.programs.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.WiringConstants.SwerveWiring;
import frc.robot.hardware.Motor;
import frc.robot.programs.LoggedRobot;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.SysIDCommands;
import frc.robot.utilities.logging.HoundLog;

@SuppressWarnings("resource")
public class SwerveSysID extends LoggedRobot {
  private Motor flDrive;
  private Motor flAngle;
  private Motor frDrive;
  private Motor frAngle;
  private Motor blDrive;
  private Motor blAngle;
  private Motor brDrive;
  private Motor brAngle;
  private Sendable targetSetter;
  private double target;

  public SwerveSysID() {
    flDrive =
        Motor.fromTalonFX(
            SwerveWiring.FRONT_LEFT_DRIVE_ID,
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
              config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(15.5);
              StatusCode status = StatusCode.StatusCodeNotInitialized;
              for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                status = motor.getConfigurator().apply(config);
              }
            },
            sim -> {},
            0,
            FeedbackController.fromPID(new PIDController(0, 0, 0), controller -> {}),
            FeedforwardController.forConstantGravity(0, 0.24571, 1.8141, 0.16798),
            Motor.TargetType.Velocity);

    frDrive =
        Motor.fromTalonFX(
            SwerveWiring.FRONT_RIGHT_DRIVE_ID,
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
              config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(15.5);
              StatusCode status = StatusCode.StatusCodeNotInitialized;
              for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                status = motor.getConfigurator().apply(config);
              }
            },
            sim -> {},
            0,
            FeedbackController.fromPID(new PIDController(0, 0, 0), controller -> {}),
            FeedforwardController.forConstantGravity(0, 0.19746, 1.7841, 0.21748),
            Motor.TargetType.Velocity);

    blDrive =
        Motor.fromTalonFX(
            SwerveWiring.BACK_LEFT_DRIVE_ID,
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
              config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(15.5);
              StatusCode status = StatusCode.StatusCodeNotInitialized;
              for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                status = motor.getConfigurator().apply(config);
              }
            },
            sim -> {},
            0,
            FeedbackController.fromPID(new PIDController(0, 0, 0), controller -> {}),
            FeedforwardController.forConstantGravity(0, 0.21224, 1.7981, 0.18871),
            Motor.TargetType.Velocity);

    brDrive =
        Motor.fromTalonFX(
            SwerveWiring.BACK_RIGHT_DRIVE_ID,
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
              config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(15.5);
              StatusCode status = StatusCode.StatusCodeNotInitialized;
              for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                status = motor.getConfigurator().apply(config);
              }
            },
            sim -> {},
            0,
            FeedbackController.fromPID(new PIDController(0, 0, 0), controller -> {}),
            FeedforwardController.forConstantGravity(0, 0.2177, 1.8156, 0.12033),
            Motor.TargetType.Velocity);

    flAngle =
        Motor.fromSparkMax(
            SwerveWiring.FRONT_LEFT_ANGLE_ID,
            false,
            motor -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
              config.encoder.positionConversionFactor(1.0 / 25).velocityConversionFactor(1.0 / 25);
              motor.configure(
                  config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            },
            sim -> {},
            new AnalogEncoder(0).get() - 0.650,
            FeedbackController.fromPID(
                new PIDController(30, 0, 10),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            FeedforwardController.forConstantGravity(0, 0.25179, 3.1033, 0.33929),
            Motor.TargetType.Position);

    frAngle =
        Motor.fromSparkMax(
            SwerveWiring.FRONT_LEFT_ANGLE_ID,
            false,
            motor -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
              config.encoder.positionConversionFactor(1.0 / 25).velocityConversionFactor(1.0 / 25);
              motor.configure(
                  config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            },
            sim -> {},
            new AnalogEncoder(1).get() - 0.906,
            FeedbackController.fromPID(
                new PIDController(30, 0, 10),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            FeedforwardController.forConstantGravity(0, 0.36617, 3.2101, 0.26453),
            Motor.TargetType.Position);

    blAngle =
        Motor.fromSparkMax(
            SwerveWiring.BACK_LEFT_ANGLE_ID,
            false,
            motor -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
              config.encoder.positionConversionFactor(1.0 / 25).velocityConversionFactor(1.0 / 25);
              motor.configure(
                  config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            },
            sim -> {},
            new AnalogEncoder(2).get() - 0.001,
            FeedbackController.fromPID(
                new PIDController(30, 0, 10),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            FeedforwardController.forConstantGravity(0, 0.37473, 3.24, 0.24393),
            Motor.TargetType.Position);

    brAngle =
        Motor.fromSparkMax(
            SwerveWiring.BACK_RIGHT_ANGLE_ID,
            false,
            motor -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
              config.encoder.positionConversionFactor(1.0 / 25).velocityConversionFactor(1.0 / 25);
              motor.configure(
                  config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            },
            sim -> {},
            new AnalogEncoder(3).get() - 0.854,
            FeedbackController.fromPID(
                new PIDController(30, 0, 10),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            FeedforwardController.forConstantGravity(0, 0.25159, 3.1999, 0.25259),
            Motor.TargetType.Position);

    targetSetter =
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("Target", () -> target, newTarget -> target = newTarget);
          }
        };
    SysIDCommands driveSysId = getDriveSysIDCommands();
    SysIDCommands angleSysId = getAngleSysIDCommands();
    SmartDashboard.putData(
        "Drive Dynamic Forward", driveSysId.dynamicForward().deadlineFor(testAnglePIDs()));
    SmartDashboard.putData(
        "Drive Dynamic Reverse", driveSysId.dynamicReverse().deadlineFor(testAnglePIDs()));
    SmartDashboard.putData(
        "Drive Quasistatic Forward", driveSysId.quasistaticForward().deadlineFor(testAnglePIDs()));
    SmartDashboard.putData(
        "Drive Quasistatic Reverse", driveSysId.quasistaticReverse().deadlineFor(testAnglePIDs()));
    SmartDashboard.putData("Angle Dynamic Forward", angleSysId.dynamicForward());
    SmartDashboard.putData("Angle Dynamic Reverse", angleSysId.dynamicReverse());
    SmartDashboard.putData("Angle Quasistatic Forward", angleSysId.quasistaticForward());
    SmartDashboard.putData("Angle Quasistatic Reverse", angleSysId.quasistaticReverse());
    SmartDashboard.putData("Angle PID Test", testAnglePIDs());
    SmartDashboard.putData("Angle Target", targetSetter);
    SmartDashboard.putData("Full Speed Ahead!", fullSpeedAhead());
  }

  @Override
  public void robotPeriodic() {
    HoundLog.log("Swerve/FLAngle", flAngle);
    HoundLog.log("Swerve/FRAngle", frAngle);
    HoundLog.log("Swerve/BLAngle", blAngle);
    HoundLog.log("Swerve/BRAngle", brAngle);
    HoundLog.log("Swerve/FLDrive", flDrive);
    HoundLog.log("Swerve/FRDrive", frDrive);
    HoundLog.log("Swerve/BLDrive", blDrive);
    HoundLog.log("Swerve/BRDrive", brDrive);
    HoundLog.log("Swerve/Target Angle", target);
    CommandScheduler.getInstance().run();
  }

  public Command fullSpeedAhead() {
    return Commands.runOnce(
            () -> {
              flDrive.setVoltage(12);
              frDrive.setVoltage(12);
              blDrive.setVoltage(12);
              brDrive.setVoltage(12);
            })
        .andThen(Commands.waitSeconds(1))
        .andThen(
            Commands.runOnce(
                () -> {
                  flDrive.setVoltage(0);
                  frDrive.setVoltage(0);
                  blDrive.setVoltage(0);
                  brDrive.setVoltage(0);
                }));
  }

  public Command testAnglePIDs() {
    return Commands.run(
            () -> {
              flAngle.setTarget(target);
              frAngle.setTarget(target);
              blAngle.setTarget(target);
              brAngle.setTarget(target);
            })
        .finallyDo(
            () -> {
              flAngle.setVoltage(0);
              frAngle.setVoltage(0);
              blAngle.setVoltage(0);
              brAngle.setVoltage(0);
            });
  }

  public SysIDCommands getDriveSysIDCommands() {
    return frDrive.getSysIDCommands("Drive SysId", 1, 2.5, 3, flDrive, blDrive, brDrive);
  }

  public SysIDCommands getAngleSysIDCommands() {
    return frAngle.getSysIDCommands("AngleSysId", 1, 5, 5, flAngle, blAngle, brAngle);
  }
}
