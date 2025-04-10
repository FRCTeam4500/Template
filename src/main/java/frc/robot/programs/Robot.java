// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.programs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.logging.HoundLog;

public class Robot extends LoggedRobot {
  private Swerve swerve;
  private Superstructure structure;
  private CommandXboxController xbox;
  private CommandJoystick stick;

  /** make a robot */
  public Robot() {
    swerve = new Swerve();
    structure = new Superstructure();
    DriverStation.silenceJoystickConnectionWarning(true);
    xbox = new CommandXboxController(2);
    stick = new CommandJoystick(1);
    swerve.setDefaultCommand(swerve.angleCentric(xbox.getHID()));

    setupDriveController();
    setupOperatorController();
    setupAuto();
  }

  private void setupOperatorController() {
    Trigger stowButton = stick.button(11);
    stowButton.onTrue(structure.stow());
  }

  private void setupDriveController() {
    Trigger onBlue =
        new Trigger(() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);
    Trigger onRed = onBlue.negate();
    Trigger faceForwards = new Trigger(() -> xbox.getRightY() < -0.5);
    Trigger faceBackwards = new Trigger(() -> xbox.getRightY() > 0.5);
    Trigger resetHeading = xbox.a();
    Trigger stow = xbox.y();

    resetHeading.and(onBlue).onTrue(swerve.resetHeading(Rotation2d.fromDegrees(0)));
    resetHeading.and(onRed).onTrue(swerve.resetHeading(Rotation2d.fromDegrees(180)));
    faceForwards.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(0)));
    faceForwards.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(180)));
    faceBackwards.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(0)));
    faceBackwards.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(180)));
    stow.onTrue(structure.stow());
  }

  private void setupAuto() {

    SendableChooser<Command> chooser = new SendableChooser<>();
    chooser.setDefaultOption("None", Commands.none());
    SmartDashboard.putData("Auto Chooser", chooser);
    RobotModeTriggers.autonomous().whileTrue(Commands.deferredProxy(chooser::getSelected));
  }

  @Override
  public void robotPeriodic() {
    double start = Timer.getFPGATimestamp();
    HoundLog.log("Swerve", swerve);
    HoundLog.log("Superstrucutre", structure);
    double loggingLoop = Timer.getFPGATimestamp() - start;

    start = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().run();
    double commandsLoop = Timer.getFPGATimestamp() - start;

    HoundLog.log("DogLog", "Logging Loop Time", loggingLoop * 1000);
    HoundLog.log("DogLog", "Commands Loop Time", commandsLoop * 1000);
    HoundLog.log("DogLog", "Total Loop Time", 1000 * (commandsLoop + loggingLoop));
  }
}
