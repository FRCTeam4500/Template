package frc.robot.utilities;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
   * Resets the feedback controller, clearing integral and derivative terms
   *
   * @param measurement The current measurement. The target of the controller is set to this!
   */
  public void reset(double measurement);

  /**
   * @param pid PID controller
   * @param config function that modifies the controller, configuring it
   * @return FeedbackController overlying the PID
   * @apiNote the setpoint will have the PID's goal position and zero velocity.
   */
  public static FeedbackController fromPID(PIDController pid, Consumer<PIDController> config) {
    pid.setTolerance(0);
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

      @Override
      public void reset(double measurement) {
        pid.calculate(measurement, measurement);
        pid.reset();
      }
    };
  }

  public static FeedbackController fromPID(
      double kP, double kI, double kD, Consumer<PIDController> config) {
    return fromPID(new PIDController(kP, kI, kD), config);
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

      @Override
      public void reset(double measurement) {
        pid.calculate(measurement, measurement);
        pid.reset(measurement);
      }
    };
  }

  public static FeedbackController fromProfiledPID(
      double kP,
      double kI,
      double kD,
      Constraints constraints,
      Consumer<ProfiledPIDController> config) {
    return fromProfiledPID(new ProfiledPIDController(kP, kI, kD, constraints), config);
  }

  public static FeedbackController empty(double tolerance) {
    return new FeedbackController() {
      private double goal;

      @Override
      public double calculate(double measurement, double goal) {
        this.goal = goal;
        return 0;
      }

      @Override
      public double getGoal() {
        return goal;
      }

      @Override
      public State getSetpoint() {
        return new State(goal, 0);
      }

      @Override
      public boolean atGoal() {
        return true;
      }

      @Override
      public void reset(double measurement) {
        this.goal = measurement;
      }
    };
  }
}
