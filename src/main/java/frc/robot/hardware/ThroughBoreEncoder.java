package frc.robot.hardware;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class ThroughBoreEncoder extends SubsystemBase implements Loggable {
  private DutyCycleEncoder encoder;
  private double angle;
  private double vel;
  private double offset;
  private boolean wasReset;

  public ThroughBoreEncoder(int channel, boolean inverted) {
    encoder = new DutyCycleEncoder(channel);
    encoder.setInverted(inverted);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(angle);
  }

  public Rotation2d getAngularVelocity() {
    return Rotation2d.fromRotations(vel);
  }

  public void setAngle(Rotation2d currentAngle) {
    double newAngle = MathUtil.inputModulus(currentAngle.getRotations(), -0.5, 0.5);
    offset += newAngle - angle;
    angle = MathUtil.inputModulus(encoder.get() - offset, -0.5, 0.5);
    wasReset = true;
  }

  @Override
  public void periodic() {
    if (wasReset) {
      angle = MathUtil.inputModulus(encoder.get() - offset, -0.5, 0.5);
      wasReset = false;
      return;
    }
    double nextAngle = MathUtil.inputModulus(encoder.get() - offset, -0.5, 0.5);
    double diff = nextAngle - angle;
    if (Math.abs(diff) > 0.5) {
      diff = -Math.signum(diff) * (1 - Math.abs(diff));
    }
    vel = diff / 0.02;
    angle = nextAngle;
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Angle", getAngle().getDegrees());
    HoundLog.log(path, "Angular Velocity", getAngularVelocity().getDegrees());
    HoundLog.log(path, "Connected", encoder.isConnected());
    HoundLog.log(path, "Offset", offset);
  }
}
