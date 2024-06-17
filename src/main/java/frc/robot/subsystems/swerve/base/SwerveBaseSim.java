package frc.robot.subsystems.swerve.base;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.subsystems.swerve.SwerveConstants.*;
public class SwerveBaseSim extends SwerveBaseIO {
    private Pose2d currentPose;
    private ChassisSpeeds currentSpeeds;
    private ChassisSpeeds targetSpeeds;
    private SwerveDriveKinematics kinematics;
    private SlewRateLimiter forwardLimiter;
    private SlewRateLimiter sidewaysLimiter;
    private SlewRateLimiter rotationalLimiter;
    public SwerveBaseSim() {
        currentPose = new Pose2d();
        currentSpeeds = new ChassisSpeeds();
        targetSpeeds = new ChassisSpeeds();
        forwardLimiter = new SlewRateLimiter(10);
        sidewaysLimiter = new SlewRateLimiter(10);
        rotationalLimiter = new SlewRateLimiter(60);
        kinematics = new SwerveDriveKinematics(
            FRONT_LEFT_MODULE_CONFIG.translation(),
            FRONT_RIGHT_MODULE_CONFIG.translation(),
            BACK_LEFT_MODULE_CONFIG.translation(),
            BACK_RIGHT_MODULE_CONFIG.translation()
        ); 
    }
    @Override
    public void setSpeeds(ChassisSpeeds speeds) {
        targetSpeeds = speeds;
    }

    @Override
    public void setPose(Pose2d pose) {
        currentPose = pose;
    }

    @Override
    public void setAngle(Rotation2d angle) {
        currentPose = new Pose2d(currentPose.getTranslation(), angle);
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return currentSpeeds;
    }

    @Override
    public Pose2d getPose() {
        return currentPose;
    }

    @Override
    public SwerveModuleState[] getStates() {
        return kinematics.toSwerveModuleStates(getSpeeds());
    }

    @Override
    public void periodic() {
        currentSpeeds.vxMetersPerSecond = forwardLimiter.calculate(targetSpeeds.vxMetersPerSecond);
        currentSpeeds.vyMetersPerSecond = sidewaysLimiter.calculate(targetSpeeds.vyMetersPerSecond);
        currentSpeeds.omegaRadiansPerSecond = rotationalLimiter.calculate(targetSpeeds.omegaRadiansPerSecond);
        var fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());
        currentPose = new Pose2d(
            currentPose.getX() + fieldRelative.vxMetersPerSecond * 0.02,
            currentPose.getY() + fieldRelative.vyMetersPerSecond * 0.02,
            currentPose.getRotation().plus(Rotation2d.fromRotations(fieldRelative.omegaRadiansPerSecond * 0.02))
        );
    }
    
}
