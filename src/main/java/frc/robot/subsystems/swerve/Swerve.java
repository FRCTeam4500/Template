package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.utilities.ExtendedMath.withHardDeadzone;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Gyro;
import frc.robot.hardware.Limelight;
import frc.robot.hardware.Limelight.PoseEstimate;
import frc.robot.utilities.ExtendedMath;
import frc.robot.utilities.GamePieceManager;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Swerve extends SubsystemBase implements Loggable {
  private Gyro gyro;
  private SwerveModule[] modules;
  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator estimator;
  private Limelight[] limelights;
  private Rotation2d targetHeading;
  private PIDController headingPID;

  public Swerve() {
    if (RobotBase.isReal()) {
      gyro = Gyro.fromNavX(navx -> {});
    } else {
      gyro = Gyro.fromSim(() -> getSpeeds().omegaRadiansPerSecond);
    }
    limelights = new Limelight[] {new Limelight("limelight-hehehe", 0)};
    modules =
        new SwerveModule[] {
          FRONT_LEFT_MODULE, FRONT_RIGHT_MODULE, BACK_LEFT_MODULE, BACK_RIGHT_MODULE
        };
    kinematics =
        new SwerveDriveKinematics(
            FRONT_LEFT_TRANSLATION,
            FRONT_RIGHT_TRANSLATION,
            BACK_LEFT_TRANSLATION,
            BACK_RIGHT_TRANSLATION);
    estimator =
        new SwerveDrivePoseEstimator(
            kinematics, gyro.getAngle(), getModulePositions(), new Pose2d());
    targetHeading = new Rotation2d();
    headingPID = new PIDController(5, 0, 0);
    headingPID.enableContinuousInput(-Math.PI, Math.PI);
    headingPID.setTolerance(Math.PI / 32, Math.PI / 32);
    headingPID.setSetpoint(0);

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      Alert alert = new Alert("READING AUTO CONFIG FILE FAILED!!", AlertType.kError);
      alert.set(true);
      alert.close();
      System.out.println(e.getMessage());
      config =
          new RobotConfig(
              68,
              6.884,
              new ModuleConfig(0.5, 6, 1.2, DCMotor.getKrakenX60(1).withReduction(5.143), 60, 1),
              FRONT_LEFT_TRANSLATION,
              FRONT_RIGHT_TRANSLATION,
              BACK_LEFT_TRANSLATION,
              BACK_RIGHT_TRANSLATION);
    }
    AutoBuilder.configure(
        estimator::getEstimatedPosition,
        this::resetPose,
        this::getSpeeds,
        this::drive,
        new PPHolonomicDriveController(new PIDConstants(5), new PIDConstants(5)),
        config,
        () -> {
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
          return alliance == Alliance.Red;
        },
        this);
  }

  public Command angleCentric(XboxController xbox) {
    return Commands.run(
            () -> {
              double speedCoefficient = Math.max(1 - xbox.getLeftTriggerAxis(), MIN_COEFFICIENT);
              Rotation2d currentHeading = estimator.getEstimatedPosition().getRotation();
              targetHeading =
                  Rotation2d.fromRadians(
                      targetHeading.getRadians()
                          - withHardDeadzone(xbox.getRightX(), 0.1)
                              * speedCoefficient
                              * MAX_SPEEDS.omegaRadiansPerSecond
                              * 0.02);
              double rotational =
                  headingPID.calculate(currentHeading.getRadians(), targetHeading.getRadians());
              if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                speedCoefficient *= -1;
              }
              double forward =
         speedCoefficient
                      * withHardDeadzone(xbox.getLeftY(), 0.1)
                      * MAX_SPEEDS.vxMetersPerSecond;
              double sideways =
                  speedCoefficient
        * withHardDeadzone(xbox.getLeftX(), 0.1)
                      * MAX_SPEEDS.vyMetersPerSecond;
              ChassisSpeeds fieldRel = new ChassisSpeeds(forward, sideways, rotational);
              drive(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRel, currentHeading));
            },
            this)
        .beforeStarting(() -> targetHeading = estimator.getEstimatedPosition().getRotation());
  }

  public Command resetHeading(Rotation2d newHeading) {
    return Commands.runOnce(
        () -> {
          resetPose(new Pose2d(estimator.getEstimatedPosition().getTranslation(), newHeading));
          targetHeading = new Rotation2d();
        });
  }

  public Command setTargetHeading(Rotation2d targetHeading) {
    return Commands.runOnce(() -> this.targetHeading = targetHeading);
  }

  private ChassisSpeeds applySkewCorrection(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    double angle = SKEW_COEFFICIENT * speeds.omegaRadiansPerSecond;
    return new ChassisSpeeds(
        Math.cos(angle) * speeds.vxMetersPerSecond - Math.sin(angle) * speeds.vyMetersPerSecond,
        Math.sin(angle) * speeds.vxMetersPerSecond + Math.cos(angle) * speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond);
  }

  public Command skewTest() {
    return run(() -> {
          ChassisSpeeds fieldRel = new ChassisSpeeds(3, 0, Math.PI);
          drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  fieldRel, estimator.getEstimatedPosition().getRotation()));
        })
        .finallyDo(() -> drive(new ChassisSpeeds()));
  }

  private void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(applySkewCorrection(speeds));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_MODULE_SPEED);
    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(states[i]);
    }
  }

  private void resetPose(Pose2d pose) {
    estimator.resetPosition(gyro.getAngle(), getModulePositions(), pose);
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
    boolean speedLimit =
        (ExtendedMath.within(getSpeeds(), new ChassisSpeeds(), new ChassisSpeeds(1, 1, 2 * Math.PI))
            || !DriverStation.isAutonomous());
    for (Limelight camera : limelights) {
      PoseEstimate estimate = camera.getPoseMT1();
      if (estimate.exists()
          && speedLimit
          && (estimate.tagCount() > 1 || estimate.averageDistance() < 4))
        estimator.addVisionMeasurement(
            estimate.pose(), Timer.getFPGATimestamp() - estimate.latencySeconds());
    }
    for (SwerveModule module : modules) {
      module.periodic();
    }
    if (RobotBase.isSimulation()) {
      GamePieceManager.updateNT(estimator.getEstimatedPosition());
    }
  }

  @Override
  public void log(String name) {
    HoundLog.log(name + "/Modules", getModuleStates());
    HoundLog.log(name + "/Speeds", getSpeeds());
    HoundLog.log(name + "/Pose", estimator.getEstimatedPosition());
    HoundLog.log(name + "/Target Heading", targetHeading);
    HoundLog.log(name + "/Gyro Angle", gyro.getAngle());
    HoundLog.log(name + "/Sideways", estimator.getEstimatedPosition().getY());
    HoundLog.log(name + "/Front Left Module", modules[0]);
    HoundLog.log(name + "/Front Right Module", modules[1]);
    HoundLog.log(name + "/Back Left Module", modules[2]);
    HoundLog.log(name + "/Back Right Module", modules[3]);
    HoundLog.log(name + "/Gyro", gyro);
  }
}
