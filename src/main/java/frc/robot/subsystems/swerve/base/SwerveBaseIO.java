package frc.robot.subsystems.swerve.base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveBaseIO {
    public abstract void setSpeeds(ChassisSpeeds speeds);
    public abstract void setPose(Pose2d pose);
    public abstract void periodic();
    public abstract void setAngle(Rotation2d angle);
    public abstract ChassisSpeeds getSpeeds();
    public abstract Pose2d getPose();
    public abstract SwerveModuleState[] getStates();
}
