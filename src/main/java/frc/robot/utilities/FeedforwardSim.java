package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

/** Models a single motor mechanism described by the given {@link FeedforwardConstants} */
public class FeedforwardSim extends SubsystemBase implements Loggable {
  private MechanismState state;
  private FeedforwardController feedforward;
  private double volts;
  private double max = Double.POSITIVE_INFINITY;
  private double min = Double.NEGATIVE_INFINITY;

  /**
   * Creates a simulated mechanism, using the given {@link FeedforwardConstants}
   *
   * @param feedforward The feedforward constants used to model the mechansim
   * @param initialPosition The initial position of the mechanism
   * @param scaleGravity Whether to scale kG based on the angle of the mechanism. If this is true,
   *     units are assumed to be degrees, with 0 being horizontal, and 0.25 pointing straight up.
   * @throws IllegalArgumentException if the feedforward's kV or kA is 0
   */
  public FeedforwardSim(FeedforwardController feedforward, double initialPosition) {
    if (!feedforward.canSimulate()) {
      throw new IllegalArgumentException("The feedforward must be able to simulate acceleration!");
    }
    this.feedforward = feedforward;
    this.state = new MechanismState(initialPosition, 0, 0);
  }

  /** Simulates the next 0.02s. If the robot is disabled, voltage is set to 0 */
  public void periodic() {
    if (DriverStation.isDisabled()) {
      volts = 0;
    }
    double acceleration = feedforward.calculateAccel(state.position, state.velocity, volts);
    double velocity = 0.02 * acceleration + state.velocity();
    double position = 0.02 * velocity + state.position();
    if (position > max) {
      position = max;
      velocity = 0;
      acceleration = 0;
    }
    if (position < min) {
      position = min;
      velocity = 0;
      acceleration = 0;
    }
    state = new MechanismState(position, velocity, acceleration);
  }

  /**
   * Sets hardstops for the motor
   *
   * @param min The minimum position of the mechanism
   * @param max The maximum position of the mechanism
   * @return this motor, for call chaining
   */
  public FeedforwardSim withHardstops(double min, double max) {
    this.min = min;
    this.max = max;
    return this;
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Voltage", volts);
    HoundLog.log(path, "State", state);
  }

  /**
   * @param volts The motor's new voltage
   */
  public void setVoltage(double volts) {
    if (Math.abs(volts) > 12) {
      volts = 12 * Math.signum(volts);
    }
    this.volts = volts;
  }

  /**
   * @return The voltage applied to the motor
   */
  public double getVoltage() {
    return volts;
  }

  /**
   * @return The mechanism's current state (position, velocity, acceleration)
   */
  public MechanismState getState() {
    return state;
  }

  /**
   * @param newPosition The updated position of the mechanism
   */
  public void resetPosition(double newPosition) {
    state = new MechanismState(newPosition, state.velocity(), state.acceleration());
  }

  /** Represents the state of a mechanism */
  public static record MechanismState(double position, double velocity, double acceleration)
      implements Loggable {
    @Override
    public void log(String path) {
      HoundLog.log(path, "Position", position);
      HoundLog.log(path, "Velocity", velocity);
      HoundLog.log(path, "Acceleration", acceleration);
    }
  }
}
