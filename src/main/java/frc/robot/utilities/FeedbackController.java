package frc.robot.utilities;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import java.util.function.Consumer;

/** Generalization of {@link ProfiledPIDController} */
public interface FeedbackController {
  /**
   * @param goal intended final position
   * @param measurement current position
   * @return next controller output
   */
  public double calculate(double measurement, double goal);

  /**
   * @return intended final position
   */
  public double getGoal();

  /**
   * @return the point towards which we are currently heading
   */
  public State getSetpoint();

  /**
   * @return are we at the goal?
   */
  public boolean atGoal();

  /**
   * @param pid PID controller
   * @param config function that modifies the controller, configuring it
   * @return FeedbackController overlying the PID
   * @apiNote the setpoint will have the PID's goal position and zero velocity.
   */
  public static FeedbackController fromPID(PIDController pid, Consumer<PIDController> config) {
    config.accept(pid);
    return new FeedbackController() {
      @Override
      public double calculate(double measurement, double goal) {
        return pid.calculate(measurement, goal);
      }

      @Override
      public double getGoal() {
        return pid.getSetpoint();
      }

      @Override
      public State getSetpoint() {
        return new State(pid.getSetpoint(), 0);
      }

      @Override
      public boolean atGoal() {
        return pid.atSetpoint();
      }
    };
  }

  /**
   * The fancy version of {@link FeedbackController#fromPID fromPID}, that allows for profiling
   * (changing the setpoint over time)
   *
   * @param pid the controller to wrap
   * @param config configuration function
   * @return wrapped PID
   */
  public static FeedbackController fromProfiledPID(
      ProfiledPIDController pid, Consumer<ProfiledPIDController> config) {
    config.accept(pid);
    return new FeedbackController() {
      @Override
      public double calculate(double measurement, double goal) {
        return pid.calculate(measurement, goal);
      }

      @Override
      public double getGoal() {
        return pid.getGoal().position;
      }

      @Override
      public State getSetpoint() {
        return pid.getSetpoint();
      }

      @Override
      public boolean atGoal() {
        return pid.atSetpoint();
      }
    };
  }
}
