package frc.robot.hardware.interfaces;

import com.pathplanner.lib.auto.PIDConstants;

public interface SwerveMotorController extends EncodedMotorController{
    public void configureForSwerve(boolean isInverted, int currentLimit, PIDConstants pid, boolean isDriveMotor);
}
