package frc.robot.hardware.interfaces;

import frc.robot.subsystems.swerve.SwerveModule.SwerveMotorConfig;

public interface SwerveMotorController extends EncodedMotorController{
    public void configureForSwerve(SwerveMotorConfig config);
}
