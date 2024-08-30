// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    public Robot() {
        new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        double loopStartTime = Timer.getFPGATimestamp();
        CommandScheduler.getInstance().run();
        DogLog.log("SystemStats/Commands Loop Time", 1000 * (Timer.getFPGATimestamp() - loopStartTime));    }
}