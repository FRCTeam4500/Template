package frc.robot.utilities;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Motor.FeedforwardConstants;
import java.util.function.BiConsumer;

public class FeedforwardSim extends SubsystemBase {
  /** function that modifies the State argument, stepping forward in time by 20 ms */
  private BiConsumer<State, Double> calc;

  /** position and velocity */
  private State state;

  /** current voltage */
  private double volts;

  /**
   * @param calc A function that steps the state forward in time (by 20 ms)
   * @param initalState The inital state of the mechanism
   * @apiNote calc should modify its arguments
   */
  public FeedforwardSim(BiConsumer<State, Double> calc, State initalState) {
    this.calc = calc;
    this.state = initalState;
  }

  /** Simulates the next 0.02s. If the robot is disabled, voltage is set to 0 */
  public void periodic() {
    if (DriverStation.isDisabled()) {
      volts = 0;
    }
    calc.accept(state, volts);
  }

  /**
   * setter function
   *
   * @param volts new voltage
   */
  public void setVoltage(double volts) {
    this.volts = volts;
  }

  /** voltage getter */
  public double getVoltage() {
    return volts;
  }

  /** position getter */
  public double getPosition() {
    return state.position;
  }

  /** velocity getter */
  public double getVelocity() {
    return state.velocity;
  }

  /** changes position */
  public void resetPosition(double newPosition) {
    state.position = newPosition;
  }

  /**
   * Creates a feedforward sim for a mechanism which gravity acts on with a constant (possibly 0)
   * force
   *
   * <p>The feedforward constants should be obtained via SysId
   *
   * @param ff The feedforward constants
   * @param initialState The inital position and velocity of the mechanism.
   * @throws IllegalArgumentException if kA or kV are 0
   */
  public static FeedforwardSim withConstantGravity(FeedforwardConstants ff, State initialState) {
    if (ff.kA() == 0 || ff.kV() == 0) {
      throw new IllegalArgumentException("kA and kV can not be 0 when making a feedforward sim!!");
    }
    return new FeedforwardSim(
        (state, volts) -> {
          double staticVolts = Math.signum(state.velocity) * ff.kS();
          double velocityVolts = state.velocity * ff.kV();
          double deltaVel = 0.02 * (volts - ff.kG() - staticVolts - velocityVolts) / ff.kA();
          double averageVel = state.velocity + deltaVel / 2;
          state.position += 0.02 * averageVel;
          state.velocity += deltaVel;
        },
        initialState);
  }

  /**
   * Creates a feedforward sim for a mechanism which gravity acts on with a force that is
   * proportional to the angle of the mechansim
   *
   * <p>The feedforward constants should be obtained via SysId
   *
   * <p><strong>Units for using this sim must be rotations and rotations/second</strong>
   *
   * @param ff The feedforward constants
   * @param initialState The inital position and velocity of the mechanism in rotations and
   *     rotations/second.
   * @throws IllegalArgumentException if kA or kV are 0
   */
  public static FeedforwardSim withScalingGravity(FeedforwardConstants ff, State initialState) {
    if (ff.kA() == 0 || ff.kV() == 0) {
      throw new IllegalArgumentException("kA and kV can not be 0 when making a feedforward sim!!");
    }
    return new FeedforwardSim(
        (state, volts) -> {
          double gravityVolts = Math.cos(state.position * 2 * Math.PI) * ff.kG();
          double staticVolts = Math.signum(state.velocity) * ff.kS();
          double velocityVolts = state.velocity * ff.kV();
          double deltaVel = 0.02 * (volts - gravityVolts - staticVolts - velocityVolts) / ff.kA();
          double averageVel = state.velocity + deltaVel / 2;
          state.position += 0.02 * averageVel;
          state.velocity += deltaVel;
        },
        initialState);
  }
}
