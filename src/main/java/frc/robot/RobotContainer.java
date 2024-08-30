// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.SwerveIO;

public class RobotContainer {
    private CommandXboxController xbox;
    private SwerveIO swerve;
    private Superstructure structure;
    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        xbox = new CommandXboxController(2);
        structure = new Superstructure();
        swerve = SwerveIO.getInstance();
        swerve.setDefaultCommand(swerve.angleCentric(xbox));
        configureButtons();
        configureLogging();
        configureAuto();
    }

    private void configureButtons() {
        Trigger onBlue = new Trigger(() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);
        Trigger onRed = onBlue.negate();
        Trigger faceForward = new Trigger(() -> xbox.getRightY() < -0.5);
        Trigger faceBackwards = new Trigger(() -> xbox.getRightY() > 0.5);
        Trigger setAsZero = xbox.a();

        setAsZero.onTrue(swerve.resetGyro());
        faceForward.and(onBlue).onTrue(swerve.targetAngle(Rotation2d.fromDegrees(0)));
        faceForward.and(onRed).onTrue(swerve.targetAngle(Rotation2d.fromDegrees(180)));
        faceBackwards.and(onBlue).onTrue(swerve.targetAngle(Rotation2d.fromDegrees(180)));
        faceBackwards.and(onRed).onTrue(swerve.targetAngle(Rotation2d.fromDegrees(0)));
    }

    private void configureLogging() {
        DogLogOptions homeOptions = new DogLogOptions(true, true, true, true, 1000);
        DogLogOptions compOptions = new DogLogOptions(false, true, true, true, 1000);
        DogLog.setEnabled(true);
        DogLog.setPdh(new PowerDistribution());
        DogLog.setOptions(homeOptions);

        Field2d field = new Field2d();
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

        Commands.run(() -> {
            DogLog.setOptions(
                DriverStation.isFMSAttached() ? compOptions : homeOptions
            );
            swerve.log();
            structure.log();
            field.setRobotPose(swerve.getState().pose());
        }).ignoringDisable(true).withName("Logging Command").schedule();
    }

    private void configureAuto() {
        SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", chooser);
        RobotModeTriggers.autonomous().whileTrue(Commands.deferredProxy(chooser::getSelected));
    }
}
