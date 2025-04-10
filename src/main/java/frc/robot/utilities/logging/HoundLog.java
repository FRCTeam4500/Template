package frc.robot.utilities.logging;

import dev.doglog.DogLog;
import edu.wpi.first.util.struct.StructSerializable;

/** A wrapper around DogLog to use for the sake of consistency and customizability */
public class HoundLog extends DogLog {
  public static void log(String key, Loggable value) {
    value.log(key);
  }

  public static void log(String path, String key, Loggable value) {
    log(path + "/" + key, value);
  }

  public static void log(String key, Loggable[] value) {
    for (Loggable loggable : value) {
      loggable.log(key);
    }
  }

  public static void log(String path, String key, Loggable[] value) {
    log(path + "/" + key, value);
  }

  public static void log(String path, String key, boolean value) {
    log(path + "/" + key, value);
  }

  public static void log(String path, String key, boolean[] value) {
    log(path + "/" + key, value);
  }

  public static void log(String path, String key, double value) {
    log(path + "/" + key, value);
  }

  public static void log(String path, String key, double[] value) {
    log(path + "/" + key, value);
  }

  public static void log(String path, String key, int value) {
    log(path + "/" + key, value);
  }

  public static void log(String path, String key, int[] value) {
    log(path + "/" + key, value);
  }

  public static void log(String path, String key, String value) {
    log(path + "/" + key, value);
  }

  public static void log(String path, String key, String[] value) {
    log(path + "/" + key, value);
  }

  public static <T extends StructSerializable> void log(String path, String key, T value) {
    log(path + "/" + key, value);
  }

  public static <T extends StructSerializable> void log(String path, String key, T[] value) {
    log(path + "/" + key, value);
  }

  public static void log(String path, String key, Enum<?> value) {
    log(path + "/" + key, value);
  }

  public static void log(String path, String key, Enum<?>[] value) {
    log(path + "/" + key, value);
  }
}
