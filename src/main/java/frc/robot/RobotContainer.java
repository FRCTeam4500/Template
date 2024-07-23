// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.utilities.EZLogger;
import frc.robot.utilities.EZLogger.Loggable;

public class RobotContainer {
    private CommandXboxController xbox;
    private SwerveIO swerve;
    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        xbox = new CommandXboxController(2);
        swerve = SwerveIO.getInstance();
        swerve.setDefaultCommand(swerve.angleCentric(xbox));
        configureButtons();
        configureLogging();
        configureAuto();
    }

    private void configureButtons() {
        Trigger rightStickUp = new Trigger(() -> xbox.getRightY() < -0.5);
        Trigger rightStickDown = new Trigger(() -> xbox.getRightY() > 0.5);
        Trigger onBlue = new Trigger(() -> 
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);

        xbox.a().onTrue(swerve.resetGyro());
        rightStickUp.and(onBlue).or(rightStickDown.and(onBlue.negate()))
            .onTrue(swerve.targetAngle(Rotation2d.fromDegrees(0)));
        rightStickUp.and(onBlue.negate()).or(rightStickDown.and(onBlue))
            .onTrue(swerve.targetAngle(Rotation2d.fromDegrees(180)));
        xbox.rightStick().onTrue(swerve.targetAngle(Rotation2d.fromDegrees(-90)));
        xbox.leftBumper().onTrue(swerve.targetAngle(Rotation2d.fromDegrees(90)));
    }

    private void configureLogging() {
        EZLogger.put("Swerve", (Loggable) swerve);
    }

    private void configureAuto() {
        SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
        EZLogger.put("Auto Chooser", chooser);
        RobotModeTriggers.autonomous().whileTrue(Commands.deferredProxy(chooser::getSelected));
    }
}
