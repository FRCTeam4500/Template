package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.utilities.ExtendedMath.withHardDeadzone;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hardware.Gyro;
import frc.robot.hardware.Limelight;
import frc.robot.hardware.Limelight.PoseEstimate;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.PoseFeedbackController;
import frc.robot.utilities.StopTilting;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

/** The subsystem that controls our drivetrain, which is known as a swerve drive. */
public class Swerve extends SubsystemBase implements Loggable {
  private Gyro gyro;
  private SwerveModule[] modules;
  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator estimator;
  private Limelight[] tagCameras;
  private Rotation2d targetHeading;
  private FeedbackController headingFeedback;
  private PoseFeedbackController poseFeedback;

  /** Creates a new {@link Swerve} using the constants defined in {@link SwerveConstants} */
  public Swerve() {
    tagCameras = new Limelight[] {};

    gyro = Gyro.fromNavX(() -> getSpeeds().omegaRadiansPerSecond, navx -> {});
    modules =
        new SwerveModule[] {
          new SwerveModule(FRONT_LEFT_DRIVE_MOTOR, FRONT_LEFT_ANGLE_MOTOR),
          new SwerveModule(FRONT_RIGHT_DRIVE_MOTOR, FRONT_RIGHT_ANGLE_MOTOR),
          new SwerveModule(BACK_LEFT_DRIVE_MOTOR, BACK_LEFT_ANGLE_MOTOR),
          new SwerveModule(BACK_RIGHT_DRIVE_MOTOR, BACK_RIGHT_ANGLE_MOTOR)
        };
    kinematics =
        new SwerveDriveKinematics(
            FRONT_LEFT_TRANSLATION,
            FRONT_RIGHT_TRANSLATION,
            BACK_LEFT_TRANSLATION,
            BACK_RIGHT_TRANSLATION);
    estimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            gyro.getAngle(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(5, 5, 5));
    StopTilting.setupKinematics(kinematics);
    StopTilting.setupBase(
        estimator::getEstimatedPosition, new Transform3d(0, 0, 0.1, Rotation3d.kZero), 39.3468644);
    targetHeading = new Rotation2d();
    headingFeedback =
        FeedbackController.fromPID(
            5,
            0,
            0,
            pid -> {
              pid.enableContinuousInput(-Math.PI, Math.PI);
              pid.setTolerance(Math.PI / 32, Math.PI / 32);
              pid.setSetpoint(0);
            });
    if (RobotBase.isReal()) {
      poseFeedback =
          new PoseFeedbackController(
              FeedbackController.fromPID(
                  1.5,
                  0,
                  0,
                  pid -> {
                    pid.setTolerance(0.02, 0.1);
                  }),
              FeedbackController.fromPID(
                  2,
                  0,
                  0,
                  pid -> {
                    pid.setTolerance(0.015, 0.1);
                  }),
              FeedbackController.fromPID(
                  6,
                  0,
                  0,
                  pid -> {
                    pid.enableContinuousInput(0, 360);
                    pid.setTolerance(2);
                  }));
    } else {
      poseFeedback =
          new PoseFeedbackController(
              FeedbackController.fromPID(
                  5,
                  0,
                  0,
                  pid -> {
                    pid.setTolerance(0.02);
                  }),
              FeedbackController.fromPID(
                  5,
                  0,
                  0,
                  pid -> {
                    pid.setTolerance(0.02);
                  }),
              FeedbackController.fromPID(
                  3,
                  0,
                  0,
                  pid -> {
                    pid.enableContinuousInput(0, 360);
                    pid.setTolerance(1);
                  }));
    }

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      HoundLog.logFault("[Swerve] Failed to read auto config...", AlertType.kError);
      System.out.println(e.getMessage());
      config =
          new RobotConfig( // TODO fix
              53.126, // robot's mass in kg
              4.954, // Robot's moment of inertia
              new ModuleConfig(0.045, 4.500, 1.1, DCMotor.getKrakenX60(1).withReduction(6), 40, 1),
              FRONT_LEFT_TRANSLATION.getY()
                  * 2 // the trackwidth of the robot (dist. from top left to top right for example)
              );
    }
    AutoBuilder.configure(
        estimator::getEstimatedPosition,
        this::resetPose,
        this::getSpeeds,
        this::drive,
        new PPHolonomicDriveController(new PIDConstants(5), new PIDConstants(1)),
        config,
        () -> {
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
          return alliance == Alliance.Red;
        },
        this);
    SmartDashboard.putData("Drive Conversion Factor", driveConversionFinder(0.25, 10));
  }

  /**
   * @param xbox The {@link XboxController} that will control the driving
   * @return a {@link Command} that drives the robot using a {@link XboxController}.
   *     <ul>
   *       <li>Translation of the robot is controlled with the left stick, field relative
   *       <li>Rotation of the robot is controlled with the right stick
   *       <li>A target angle can be set using {@link #setTargetHeading}, which the robot will then
   *           turn to.
   *     </ul>
   */
  public Command angleCentric(XboxController xbox) {
    return Commands.run(
            () -> {
              drive(calculateVelRobotRel(xbox));
            },
            this)
        .beforeStarting(() -> targetHeading = estimator.getEstimatedPosition().getRotation())
        .withName("Angle Centric");
  }

  /**
   * @param xbox The {@link XboxController} that will control the driving
   * @return a {@link Command} that drives the robot using a {@link XboxController}.
   *     <ul>
   *       <li>Translation of the robot is controlled with the left stick, robot relative
   *       <li>Rotation of the robot is controlled with the right stick
   *     </ul>
   */
  public Command robotCentric(XboxController xbox) {
    return Commands.run(
            () -> {
              double coefficient = Math.max(1 - xbox.getLeftTriggerAxis(), SLOWEST_COEFFICIENT);
              drive(
                  new ChassisSpeeds(
                      coefficient
                          * withHardDeadzone(-xbox.getLeftY(), 0.1)
                          * MAX_TELEOP_SPEEDS.vxMetersPerSecond,
                      coefficient
                          * withHardDeadzone(-xbox.getLeftX(), 0.1)
                          * MAX_TELEOP_SPEEDS.vyMetersPerSecond,
                      coefficient
                          * withHardDeadzone(-xbox.getRightX(), 0.1)
                          * MAX_TELEOP_SPEEDS.omegaRadiansPerSecond));
            },
            this)
        .withName("Robot Centric");
  }

  /*
   * @param target pose on the field which the robot should try to move toward.
   * @return a {@link Command} that drives the robot to the provided target pose.
   */
  public Command poseCentric(Pose2d target) {
    return Commands.run(
            () -> {
              Pose2d current = estimator.getEstimatedPosition();
              ChassisSpeeds speeds = poseFeedback.calculate(current, target);
              drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, current.getRotation()));
            },
            this)
        .beforeStarting(
            () -> {
              poseFeedback.reset(estimator.getEstimatedPosition());
            })
        .until(() -> poseFeedback.atTarget())
        .withName("Pose Centric");
  }

  /**
   * Updates the heading of the robot
   *
   * @param newHeading The robots new heading
   * @return A {@link Command} that can be bound to a {@link Trigger}
   */
  public Command resetHeading(Rotation2d newHeading) {
    return Commands.runOnce(
        () -> {
          resetPose(new Pose2d(estimator.getEstimatedPosition().getTranslation(), newHeading));
          targetHeading = newHeading;
        });
  }

  /**
   * Sets the target heading for the robot in tele-op only
   *
   * @param targetHeading The target heading
   * @return A {@link Command} that can be bound to a {@link Trigger}
   */
  public Command setTargetHeading(Rotation2d targetHeading) {
    return Commands.runOnce(() -> this.targetHeading = targetHeading);
  }

  public Command driveConversionFinder(double speed, double duration) {
    class CharacterizationState {
      double[] startPositions = new double[4];
      double[] endPositions = new double[4];
      double startAngle = 0;
      double endAngle = 0;
    }
    CharacterizationState state = new CharacterizationState();
    return Commands.runOnce(
            () -> {
              drive(new ChassisSpeeds(0, 0, speed));
            },
            this)
        .andThen(Commands.waitSeconds(1))
        .andThen(
            Commands.runOnce(
                () -> {
                  state.startAngle = gyro.getAngle().getRadians();
                  state.startPositions =
                      new double[] {
                        modules[0].getCurrentPosition().distanceMeters,
                        modules[1].getCurrentPosition().distanceMeters,
                        modules[2].getCurrentPosition().distanceMeters,
                        modules[3].getCurrentPosition().distanceMeters
                      };
                }))
        .andThen(Commands.waitSeconds(duration))
        .andThen(
            Commands.runOnce(
                () -> {
                  state.endAngle = gyro.getAngle().getRadians();
                  state.endPositions =
                      new double[] {
                        modules[0].getCurrentPosition().distanceMeters,
                        modules[1].getCurrentPosition().distanceMeters,
                        modules[2].getCurrentPosition().distanceMeters,
                        modules[3].getCurrentPosition().distanceMeters
                      };
                  double gyroDelta = Math.abs(state.endAngle - state.startAngle);
                  for (int i = 0; i < modules.length; i++) {
                    double wheelDelta = Math.abs(state.endPositions[i] - state.startPositions[i]);
                    wheelDelta /= SwerveConstants.BACK_LEFT_TRANSLATION.getNorm();
                    switch (i) {
                      case 0:
                        System.out.println(
                            "Front Left Drive Motor New Conversion Factor: "
                                + FRONT_LEFT_CONFIG.driveConversionFactor()
                                    * (gyroDelta / wheelDelta));
                        break;
                      case 1:
                        System.out.println(
                            "Front Right Drive Motor New Conversion Factor: "
                                + FRONT_RIGHT_CONFIG.driveConversionFactor()
                                    * (gyroDelta / wheelDelta));
                        break;
                      case 2:
                        System.out.println(
                            "Back Left Drive Motor New Conversion Factor: "
                                + BACK_LEFT_CONFIG.driveConversionFactor()
                                    * (gyroDelta / wheelDelta));
                        break;
                      case 3:
                        System.out.println(
                            "Back Right Drive Motor New Conversion Factor: "
                                + BACK_RIGHT_CONFIG.driveConversionFactor()
                                    * (gyroDelta / wheelDelta));
                        break;
                      default:
                        break;
                    }
                  }
                }))
        .withName("Drive Conversion Factor Finder");
  }

  /**
   * @return a {@link Command} that moves the robot back at 2m/s for 0.25s
   */
  public Command backup() {
    return Commands.run(
            () -> {
              drive(new ChassisSpeeds(-2, 0, 0));
            },
            this)
        .withTimeout(.25)
        .withName("Backup");
  }

  /**
   * @param pushDirection the direction to face the wheels
   * @return a {@link Command} that turns all wheels to face provided direction
   */
  public Command makePushable(Rotation2d pushDirection) {
    return Commands.run(
            () -> {
              FRONT_LEFT_ANGLE_MOTOR.setTarget(pushDirection.getDegrees());
              FRONT_RIGHT_ANGLE_MOTOR.setTarget(pushDirection.getDegrees());
              BACK_LEFT_ANGLE_MOTOR.setTarget(pushDirection.getDegrees());
              BACK_RIGHT_ANGLE_MOTOR.setTarget(pushDirection.getDegrees());
            },
            this)
        .beforeStarting(
            () -> {
              FRONT_LEFT_DRIVE_MOTOR.setTarget(0);
              FRONT_RIGHT_DRIVE_MOTOR.setTarget(0);
              BACK_LEFT_DRIVE_MOTOR.setTarget(0);
              BACK_RIGHT_DRIVE_MOTOR.setTarget(0);
            })
        .withName("Pushable: " + pushDirection.getDegrees() + " Degrees");
  }

  /**
   * @return a {@link Command} that faces all wheels to the middle of the robot (to minimize
   *     movement)
   */
  public Command xLock() {
    return Commands.run(
            () -> {
              FRONT_LEFT_ANGLE_MOTOR.setTarget(45);
              FRONT_RIGHT_ANGLE_MOTOR.setTarget(-45);
              BACK_LEFT_ANGLE_MOTOR.setTarget(-45);
              BACK_RIGHT_ANGLE_MOTOR.setTarget(45);
            },
            this)
        .withName("Wheel Lock");
  }

  /**
   * @param xbox The {@link XboxController} that will control the driving
   * @return The speeds (robot-relative) of the robot calculated from xbox inputs.
   */
  private ChassisSpeeds calculateVelRobotRel(XboxController xbox) {
    double speedCoefficient = Math.max(1 - xbox.getLeftTriggerAxis(), SLOWEST_COEFFICIENT);
    Rotation2d currentHeading = estimator.getEstimatedPosition().getRotation();
    targetHeading =
        Rotation2d.fromRadians(
            targetHeading.getRadians()
                - withHardDeadzone(xbox.getRightX(), 0.1)
                    * speedCoefficient
                    * MAX_TELEOP_SPEEDS.omegaRadiansPerSecond
                    * 0.02);
    double rotational =
        headingFeedback.calculate(currentHeading.getRadians(), targetHeading.getRadians());
    if (headingFeedback.atGoal()) {
      rotational = 0;
    }
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      speedCoefficient *= -1;
    }
    double forward =
        speedCoefficient
            * withHardDeadzone(xbox.getLeftY(), 0.1)
            * MAX_TELEOP_SPEEDS.vxMetersPerSecond;
    double sideways =
        speedCoefficient
            * withHardDeadzone(xbox.getLeftX(), 0.1)
            * MAX_TELEOP_SPEEDS.vyMetersPerSecond;
    ChassisSpeeds fieldRel = new ChassisSpeeds(forward, sideways, rotational);
    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRel, currentHeading);
  }

  private ChassisSpeeds applySkewCorrection(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    double angle = SKEW_COEFFICIENT * speeds.omegaRadiansPerSecond;
    return new ChassisSpeeds(
        Math.cos(angle) * speeds.vxMetersPerSecond - Math.sin(angle) * speeds.vyMetersPerSecond,
        Math.sin(angle) * speeds.vxMetersPerSecond + Math.cos(angle) * speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond);
  }

  /**
   * moves the robot according to provided speeds.
   *
   * @param speeds the target speeds (robot-relative) of the robot
   */
  private void drive(ChassisSpeeds speeds) {
    double coefficient = MAX_ROBOT_SPEEDS.vxMetersPerSecond / Math.abs(speeds.vxMetersPerSecond);
    if (coefficient < 1) {
      speeds =
          new ChassisSpeeds(
              speeds.vxMetersPerSecond * coefficient,
              speeds.vyMetersPerSecond * coefficient,
              speeds.omegaRadiansPerSecond * coefficient);
    }
    coefficient = MAX_ROBOT_SPEEDS.vyMetersPerSecond / Math.abs(speeds.vyMetersPerSecond);
    if (coefficient < 1) {
      speeds =
          new ChassisSpeeds(
              speeds.vxMetersPerSecond * coefficient,
              speeds.vyMetersPerSecond * coefficient,
              speeds.omegaRadiansPerSecond * coefficient);
    }
    coefficient = MAX_ROBOT_SPEEDS.omegaRadiansPerSecond / Math.abs(speeds.omegaRadiansPerSecond);
    if (coefficient < 1) {
      speeds =
          new ChassisSpeeds(
              speeds.vxMetersPerSecond * coefficient,
              speeds.vyMetersPerSecond * coefficient,
              speeds.omegaRadiansPerSecond * coefficient);
    }
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_MODULE_SPEED);
    speeds = kinematics.toChassisSpeeds(states);
    speeds = StopTilting.limitAccel(speeds);
    HoundLog.log("Swerve", "Target Speed", speeds);
    states = kinematics.toSwerveModuleStates(applySkewCorrection(speeds));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_MODULE_SPEED);
    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(states[i]);
    }
  }

  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  private void resetPose(Pose2d pose) {
    estimator.resetPosition(gyro.getAngle(), getModulePositions(), pose);
    targetHeading = pose.getRotation();
  }

  private ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getCurrentState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {

      states[i] = modules[i].getCurrentPosition();
    }
    return states;
  }

  @Override
  public void periodic() {
    estimator.update(gyro.getAngle(), getModulePositions());
    for (Limelight camera : tagCameras) {
      PoseEstimate estimate = camera.getPoseMT1();
      if (estimate.exists()
          && (estimate.tagCount() > 1 // sees more than one tag
              || estimate.averageDistance() < 2 // within 2m
              || DriverStation.isDisabled()) // robot disabled
          && camera.isEnabled()) {
        estimator.addVisionMeasurement(
            estimate.pose(),
            Timer.getFPGATimestamp()
                - estimate.latencySeconds()); // update estimator with vision-determined pose
      }
    }
    for (SwerveModule module : modules) {
      module.periodic();
    }
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Modules", getModuleStates());
    HoundLog.log(path, "Speeds", getSpeeds());
    HoundLog.log(path, "Pose", estimator.getEstimatedPosition());
    HoundLog.log(path, "Target Heading", targetHeading);
    HoundLog.log(path, "Gyro Angle", gyro.getAngle());
    HoundLog.log(path, "Front Left Module", modules[0]);
    HoundLog.log(path, "Front Right Module", modules[1]);
    HoundLog.log(path, "Back Left Module", modules[2]);
    HoundLog.log(path, "Back Right Module", modules[3]);
    HoundLog.log(path, "Gyro", gyro);
    HoundLog.log(path, "At Target Pose", poseFeedback.atTarget());
    for (Limelight camera : tagCameras) {
      HoundLog.log(path, camera.getName(), camera);
    }
  }
}
