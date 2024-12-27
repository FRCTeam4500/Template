package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.Motor;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class SwerveModule implements Loggable {
  private Motor drive;
  private Motor angle;
  private SwerveModuleState targetState;

  public SwerveModule(Motor drive, Motor angle) {
    this.drive = drive;
    this.angle = angle;
    targetState =
        new SwerveModuleState(
            drive.getVelocity(),
            Rotation2d.fromRotations(MathUtil.inputModulus(angle.getPosition(), 0, 1)));
  }

  public void setTargetState(SwerveModuleState state) {
    targetState = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
    targetState.optimize(
        Rotation2d.fromRotations(MathUtil.inputModulus(angle.getPosition(), 0, 1)));
  }
  ;

  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(
        drive.getVelocity(),
        Rotation2d.fromRotations(MathUtil.inputModulus(angle.getPosition(), 0, 1)));
  }
  ;

  public SwerveModulePosition getCurrentPosition() {
    return new SwerveModulePosition(
        drive.getPosition(),
        Rotation2d.fromRotations(MathUtil.inputModulus(angle.getPosition(), 0, 1)));
  }

  public void periodic() {
    targetState.cosineScale(getCurrentState().angle);
    drive.setTarget(targetState.speedMetersPerSecond);
    angle.setTarget(targetState.angle.getRotations());
  }
  ;

  @Override
  public void log(String path) {
    HoundLog.log(path, "Current State", getCurrentState());
    HoundLog.log(path, "Target State", targetState);
    HoundLog.log(path, "Drive Motor", drive);
    HoundLog.log(path, "Angle Motor", angle);
  }
}
