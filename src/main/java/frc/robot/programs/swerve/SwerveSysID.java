package frc.robot.programs.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.programs.LoggedRobot;
import frc.robot.utilities.SysIDCommands;
import frc.robot.utilities.logging.HoundLog;
public class SwerveSysID extends LoggedRobot {
  private Sendable targetSetter;
  private double target;

  public SwerveSysID() {
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
    SmartDashboard.putData("Full Speed Ahead!", fullSpeedAhead().deadlineFor(testAnglePIDs()));
  }

  @Override
  public void robotPeriodic() {
    HoundLog.log("Swerve/FLAngle", FRONT_LEFT_ANGLE_MOTOR);
    HoundLog.log("Swerve/FRAngle", FRONT_RIGHT_ANGLE_MOTOR);
    HoundLog.log("Swerve/BLAngle", BACK_LEFT_ANGLE_MOTOR);
    HoundLog.log("Swerve/BRAngle", BACK_RIGHT_ANGLE_MOTOR);
    HoundLog.log("Swerve/FLDrive", FRONT_LEFT_DRIVE_MOTOR);
    HoundLog.log("Swerve/FRDrive", FRONT_RIGHT_DRIVE_MOTOR);
    HoundLog.log("Swerve/BLDrive", BACK_LEFT_DRIVE_MOTOR);
    HoundLog.log("Swerve/BRDrive", BACK_RIGHT_DRIVE_MOTOR);
    HoundLog.log("Swerve/Target Angle", target);
    CommandScheduler.getInstance().run();
  }

  public Command fullSpeedAhead() {
    return Commands.runOnce(
            () -> {
              FRONT_LEFT_DRIVE_MOTOR.setVoltage(12);
              FRONT_RIGHT_DRIVE_MOTOR.setVoltage(12);
              BACK_LEFT_DRIVE_MOTOR.setVoltage(12);
              BACK_RIGHT_DRIVE_MOTOR.setVoltage(12);
            })
        .andThen(Commands.waitSeconds(1))
        .andThen(
            Commands.runOnce(
                () -> {
                  FRONT_LEFT_DRIVE_MOTOR.setVoltage(0);
                  FRONT_RIGHT_DRIVE_MOTOR.setVoltage(0);
                  BACK_LEFT_DRIVE_MOTOR.setVoltage(0);
                  BACK_RIGHT_DRIVE_MOTOR.setVoltage(0);
                }));
  }

  public Command testAnglePIDs() {
    return Commands.run(
            () -> {
              FRONT_LEFT_ANGLE_MOTOR.setTarget(target);
              FRONT_RIGHT_ANGLE_MOTOR.setTarget(target);
              BACK_LEFT_ANGLE_MOTOR.setTarget(target);
              BACK_RIGHT_ANGLE_MOTOR.setTarget(target);
            })
        .finallyDo(
            () -> {
              FRONT_LEFT_ANGLE_MOTOR.setVoltage(0);
              FRONT_RIGHT_ANGLE_MOTOR.setVoltage(0);
              BACK_LEFT_ANGLE_MOTOR.setVoltage(0);
              BACK_RIGHT_ANGLE_MOTOR.setVoltage(0);
            });
  }

  public SysIDCommands getDriveSysIDCommands() {
    return FRONT_LEFT_DRIVE_MOTOR.getSysIDCommands("Drive SysId", 1, 2.5, 3, FRONT_RIGHT_DRIVE_MOTOR, BACK_LEFT_DRIVE_MOTOR, BACK_RIGHT_DRIVE_MOTOR);
  }

  public SysIDCommands getAngleSysIDCommands() {
    return FRONT_LEFT_ANGLE_MOTOR.getSysIDCommands("AngleSysId", 1, 5, 5, FRONT_LEFT_ANGLE_MOTOR, BACK_LEFT_ANGLE_MOTOR, BACK_RIGHT_ANGLE_MOTOR);
  }
}
