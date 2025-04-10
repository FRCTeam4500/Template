package frc.robot.subsystems.orchestra;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.File;
import java.io.FilenameFilter;
import java.util.ArrayList;

public class Orc {
  private static String[] musicChoices;
  private static ArrayList<TalonFX> motors;
  private static SendableChooser<String> songSelector;
  private static Orchestra orchestra;

  static {
    motors = new ArrayList<>();
    orchestra = new Orchestra();
    songSelector = new SendableChooser<String>();
    songSelector.setDefaultOption("None", null);
    musicChoices =
        new File(Filesystem.getDeployDirectory(), "music")
            .list(
                new FilenameFilter() {
                  public boolean accept(File dir, String name) {
                    return name.toLowerCase().endsWith(".chrp");
                  }
                });
    for (String fileName : musicChoices) {
      songSelector.addOption(fileName.split("\\.(?=[^\\.]+$)")[0], fileName);
    }
    SmartDashboard.putData("Song Selector", songSelector);
  }

  public static void addMotor(TalonFX motor) {
    motors.add(motor);
  }

  public static Command startSinging() {
    return Commands.runOnce(
            () -> {
              for (TalonFX motor : motors) orchestra.addInstrument(motor);
              orchestra.loadMusic("music/" + songSelector.getSelected());
            })
        .andThen(Commands.waitSeconds(0.25))
        .andThen(Commands.runOnce(() -> orchestra.play()))
        .ignoringDisable(true);
  }

  public static Command stopSinging() {
    return Commands.runOnce(
        () -> {
          orchestra.stop();
          orchestra.clearInstruments();
        });
  }
}
