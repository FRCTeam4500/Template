package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.LoggedTunableNumber;

public class MoveToPoseCommand extends CommandBase {

    private SwerveDrive swerve;
    private Supplier<Pose2d> targetPoseSupplier;

    private Translation2d lastSetpointTranslation;
    private boolean running;
    private double driveErrorAbs;
    private double thetaErrorAbs;

    private ProfiledPIDController driveController = new ProfiledPIDController(
            0,
            0,
            0,
            new Constraints(
                    SwerveConstants.MAX_LINEAR_SPEED_MPS,
                    4),
            0.02);

    private ProfiledPIDController thetaController = new ProfiledPIDController(
            0,
            0,
            0,
            new Constraints(
                    Math.PI, // TODO: CHANGE
                    Math.PI/2),
            0.02);

    private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("DriveToPose/DriveKp");
    private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("DriveToPose/DriveKd");
    private static final LoggedTunableNumber thetaKp = new LoggedTunableNumber("DriveToPose/ThetaKp");
    private static final LoggedTunableNumber thetaKd = new LoggedTunableNumber("DriveToPose/ThetaKd");
    private static final LoggedTunableNumber driveMaxVelocity = new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
    private static final LoggedTunableNumber driveMaxAcceleration = new LoggedTunableNumber(
            "DriveToPose/DriveMaxAcceleration");
    private static final LoggedTunableNumber thetaMaxVelocity = new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
    private static final LoggedTunableNumber thetaMaxAcceleration = new LoggedTunableNumber(
            "DriveToPose/ThetaMaxAcceleration");
    private static final LoggedTunableNumber driveTolerance = new LoggedTunableNumber("DriveToPose/DriveTolerance");
    private static final LoggedTunableNumber thetaTolerance = new LoggedTunableNumber("DriveToPose/ThetaTolerance");
    private static final LoggedTunableNumber ffMinRadius = new LoggedTunableNumber("DriveToPose/FFMinRadius");
    private static final LoggedTunableNumber ffMaxRadius = new LoggedTunableNumber("DriveToPose/FFMinRadius");

    static {
        driveKp.initDefault(2.0);
        driveKd.initDefault(0.0);
        thetaKp.initDefault(5.0);
        thetaKd.initDefault(0.0);
        driveMaxVelocity.initDefault(Units.inchesToMeters(150.0));
        driveMaxAcceleration.initDefault(Units.inchesToMeters(95.0));
        thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
        thetaMaxAcceleration.initDefault(Units.degreesToRadians(720.0));
        driveTolerance.initDefault(0.01);
        thetaTolerance.initDefault(Units.degreesToRadians(1.0));
        ffMinRadius.initDefault(0.2);
        ffMaxRadius.initDefault(0.8);
    }

    /**
     * 
     * @param targetPose
     * @param targetSpeed Meters Per Second
     */
    public MoveToPoseCommand(Pose2d targetPose) {

        this(() -> targetPose);

    }

    public MoveToPoseCommand(Supplier<Pose2d> targetPoseSupplier) {
        this.targetPoseSupplier = targetPoseSupplier;
        this.swerve = SwerveDrive.getInstance();
        addRequirements(swerve);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        var currentPose = swerve.getEstimatorPose();
        driveController.reset(
                currentPose.getTranslation().getDistance(targetPoseSupplier.get().getTranslation()),
                Math.min(
                        0.0,
                        -new Translation2d(swerve.getChassisSpeeds().vxMetersPerSecond,
                                swerve.getChassisSpeeds().vyMetersPerSecond)
                                .rotateBy(
                                        targetPoseSupplier
                                                .get()
                                                .getTranslation()
                                                .minus(swerve.getEstimatorPose().getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        thetaController.reset(currentPose.getRotation().getRadians(), swerve.getChassisSpeeds().omegaRadiansPerSecond);
        lastSetpointTranslation = swerve.getEstimatorPose().getTranslation();
    }

    @Override
    public void execute() {

        running = true;

        // Update from tunable numbers
        if (driveMaxVelocity.hasChanged(hashCode())
                || driveMaxAcceleration.hasChanged(hashCode())
                || driveTolerance.hasChanged(hashCode())
                || thetaMaxVelocity.hasChanged(hashCode())
                || thetaMaxAcceleration.hasChanged(hashCode())
                || thetaTolerance.hasChanged(hashCode())
                || driveKp.hasChanged(hashCode())
                || driveKd.hasChanged(hashCode())
                || thetaKp.hasChanged(hashCode())
                || thetaKd.hasChanged(hashCode())) {
            driveController.setP(driveKp.get());
            driveController.setD(driveKd.get());
            driveController.setConstraints(
                    new Constraints(
                            driveMaxVelocity.get(),
                            driveMaxAcceleration.get()));
            driveController.setTolerance(driveTolerance.get());
            thetaController.setP(thetaKp.get());
            thetaController.setD(thetaKd.get());
            thetaController.setConstraints(
                    new Constraints(
                            thetaMaxVelocity.get(),
                            thetaMaxAcceleration.get()));
            thetaController.setTolerance(thetaTolerance.get());
        }

        // Get current and target pose
        Pose2d currentPose = swerve.getEstimatorPose();
        Pose2d targetPose = targetPoseSupplier.get();

        // Calculate drive speed
        double currentDistance = currentPose.getTranslation().getDistance(targetPoseSupplier.get().getTranslation());
        double ffScaler = MathUtil.clamp(
                (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity);
        double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance())
            driveVelocityScalar = 0.0;
        lastSetpointTranslation = new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(
                        translationToTransform(driveController.getSetpoint().position, 0.0))
                .getTranslation();

        // Calculate theta speed
        double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                        currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance())
            thetaVelocity = 0.0;

        // Command speeds
        Translation2d driveVelocity = new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(translationToTransform(driveVelocityScalar, 0.0))
                .getTranslation();
        swerve.driveRobotCentric(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

        // Log data
        Logger.getInstance().recordOutput("DriveToPose/DistanceMeasured", currentDistance);
        Logger.getInstance()
                .recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
        Logger.getInstance()
                .recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
        Logger.getInstance()
                .recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
        Logger.getInstance()
                .recordOutput(
                        "Odometry/DriveToPoseSetpoint",
                        new Pose2d(
                                lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
        Logger.getInstance().recordOutput("Odometry/DriveToPoseGoal", targetPose);

    }

    private static Transform2d translationToTransform(double x, double y) {
        return new Transform2d(new Translation2d(x, y), new Rotation2d());
    }

    @Override
    public boolean isFinished() {
        
        if (
            atGoal()
            || withinTolerance(driveTolerance.get(), Rotation2d.fromDegrees(thetaTolerance.get()))
        ) {
            return true;
        }

        return false;
    }

    /** Checks if the robot is stopped at the final pose. */
    public boolean atGoal() {
        return running && driveController.atGoal() && thetaController.atGoal();
    }

    /**
     * Checks if the robot pose is within the allowed drive and theta tolerances.
     */
    public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
        return running
                && Math.abs(driveErrorAbs) < driveTolerance
                && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
    }

    @Override
    public void end(boolean interrupted) {
        running = false;
        swerve.driveRobotCentric(new ChassisSpeeds());
    }

}
