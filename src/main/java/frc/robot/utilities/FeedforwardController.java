package frc.robot.utilities;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public interface FeedforwardController {
  public double calculateVoltage(double position, double velocity, double acceleration);

  public double calcuateVoltage(double position, double direction);

  public double calculateAccel(double position, double velocity, double voltage);

  public boolean canSimulate();

  public static FeedforwardController forNone() {
    return new FeedforwardController() {
      @Override
      public double calculateVoltage(double position, double velocity, double acceleration) {
        return 0;
      }

      @Override
      public double calcuateVoltage(double position, double direction) {
        return 0;
      }

      @Override
      public double calculateAccel(double position, double velocity, double voltage) {
        return 0;
      }

      @Override
      public boolean canSimulate() {
        return false;
      }
    };
  }

  public static FeedforwardController forConstantGravity(
      double kG, double kS, double kV, double kA) {
    return new FeedforwardController() {
      @Override
      public double calculateVoltage(double position, double velocity, double acceleration) {
        return kG + kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
      }

      @Override
      public double calcuateVoltage(double position, double direction) {
        return kG + kS * Math.signum(direction);
      }

      @Override
      public double calculateAccel(double position, double velocity, double voltage) {
        return (voltage - kG - kS * Math.signum(velocity) - kV * velocity) / kA;
      }

      @Override
      public boolean canSimulate() {
        return !(kA == 0 || kV == 0);
      }
    };
  }

  public static FeedforwardController forArmGravity(double kG, double kS, double kV, double kA) {
    return new FeedforwardController() {
      @Override
      public double calculateVoltage(double position, double velocity, double acceleration) {
        return kG * Math.cos(Math.toRadians(position))
            + kS * Math.signum(velocity)
            + kV * velocity
            + kA * acceleration;
      }

      @Override
      public double calcuateVoltage(double position, double direction) {
        return kG * Math.cos(Math.toRadians(position)) + kS * Math.signum(direction);
      }

      @Override
      public double calculateAccel(double position, double velocity, double voltage) {
        return (voltage
                - kG * Math.cos(Math.toRadians(position))
                - kS * Math.signum(velocity)
                - kV * velocity)
            / kA;
      }

      @Override
      public boolean canSimulate() {
        return !(kA == 0 || kV == 0);
      }
    };
  }

  public static FeedforwardController forWeirdGravity(
      InterpolatingDoubleTreeMap gravityVolts, double kS, double kV, double kA) {
    return new FeedforwardController() {
      @Override
      public double calculateVoltage(double position, double velocity, double acceleration) {
        return gravityVolts.get(position)
            + kS * Math.signum(velocity)
            + kV * velocity
            + kA * acceleration;
      }

      @Override
      public double calcuateVoltage(double position, double direction) {
        return gravityVolts.get(position) + kS * Math.signum(direction);
      }

      @Override
      public double calculateAccel(double position, double velocity, double voltage) {
        return (voltage - gravityVolts.get(position) - kS * Math.signum(velocity) - kV * velocity)
            / kA;
      }

      @Override
      public boolean canSimulate() {
        return !(kA == 0 || kV == 0);
      }
    };
  }
}
