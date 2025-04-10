package frc.robot.hardware;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * Interface for a standard 2d gyro
 * <pre>
 * // Example Usage
 * Gyro gyro;
 * if (RobotBase.isReal()) {
 *     gyro = Gyro.fromNavX(gyro -> {}); // Get the onboard navX, no configuring
 * } else {
 *     gyro = Gyro.fromSim(() -> getSpeeds().omegaRadiansPerSecond);
 *     // Tell the simulated gyro to turn how fast the robot is turning
 * }
 * double angle = gyro.getAngle().getDegrees();
 * double angularVelocity = gyro.getAngularVelocity().getDegrees();
 */
public interface Gyro extends Loggable {
  /**
   * @return Returns the angle of the gyro. Note that this is continous, so it will go from 360 to
   *     361, not back to 1. Also, counterclockwise is positive, in accordance with the WPI
   *     coordinate system.
   */
  public Rotation2d getAngle();

  /**
   * @return Returns the angular velocity of the gyro (per second). Counterclockwise positive
   */
  public Rotation2d getAngularVelocity();

  /**
   * @param config Method to configure the navX
   * @return the navX on the RIO wrapped as a {@link Gyro}
   */
  public static Gyro fromNavX(DoubleSupplier radiansPerSecond, Consumer<AHRS> config) {
    if (RobotBase.isSimulation()) {
      return fromSim(radiansPerSecond);
    }
    AHRS navx = new AHRS(NavXComType.kMXP_SPI);
    config.accept(navx);
    Trigger connected = new Trigger(navx::isConnected);
    connected.onFalse(
        Commands.runOnce(() -> HoundLog.logFault("Gyro Disconnected...", AlertType.kError))
            .ignoringDisable(true));
    connected.onTrue(
        Commands.runOnce(() -> HoundLog.clearFault("Gyro Disconnected...")).ignoringDisable(true));
    return new Gyro() {
      double lastAngle = navx.getRotation2d().getRadians();

      @Override
      public void log(String path) {
        HoundLog.log(path, "Connected", navx.isConnected());
        HoundLog.log(path, "Pitch", navx.getPitch());
        HoundLog.log(path, "Roll", navx.getRoll());
        HoundLog.log(path, "Angle", navx.getYaw());
        if (!navx.isConnected()) {
          lastAngle += radiansPerSecond.getAsDouble() * 0.02;
        } else {
          lastAngle = getAngle().getRadians();
        }
      }

      @Override
      public Rotation2d getAngle() {
        if (navx.isConnected()) {
          return navx.getRotation2d();
        } else {
          return Rotation2d.fromRadians(lastAngle);
        }
      }

      @Override
      public Rotation2d getAngularVelocity() {
        if (navx.isConnected()) {
          return Rotation2d.fromDegrees(-navx.getRate());
        } else {
          return Rotation2d.fromRadians(radiansPerSecond.getAsDouble());
        }
      }
    };
  }

  /**
   * @param radiansPerSecond A method that returns how fast the gyro should turn
   * @return a simulated gyro
   */
  public static Gyro fromSim(DoubleSupplier radiansPerSecond) {
    class GyroSim extends SubsystemBase implements Gyro {
      double angle = 0;

      @Override
      public void log(String path) {
        HoundLog.log(path, "Angle", Math.toDegrees(angle));
      }

      @Override
      public Rotation2d getAngle() {
        return Rotation2d.fromRadians(angle);
      }

      @Override
      public Rotation2d getAngularVelocity() {
        return Rotation2d.fromRadians(radiansPerSecond.getAsDouble());
      }

      @Override
      public void periodic() {
        angle += radiansPerSecond.getAsDouble() * 0.02;
      }
    }
    return new GyroSim();
  }
}
