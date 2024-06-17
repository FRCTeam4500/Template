// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.utilities.EZLogger;
import frc.robot.utilities.EZLogger.Loggable;

public class RobotContainer {
    private CommandXboxController xbox;
    private SwerveIO swerve;
    public RobotContainer() {
        xbox = new CommandXboxController(2);
        swerve = SwerveIO.getInstance();
        swerve.setDefaultCommand(swerve.angleCentric(
            xbox,
            () -> xbox.getRightY() < -0.5,
            () -> xbox.getRightY() > 0.5,
            () -> xbox.getHID().getRightStickButton(),
            () -> xbox.getHID().getLeftBumper()
        ));
        configureButtons();
        configureLogging();
        configureAuto();
    }

    private void configureButtons() {
        xbox.a().onTrue(swerve.resetGyro());
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
