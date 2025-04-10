package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PoseFeedbackController {
  private FeedbackController forward;
  private FeedbackController sideways;
  private FeedbackController heading;

  public PoseFeedbackController(
      FeedbackController forward, FeedbackController sideways, FeedbackController headingDegrees) {
    this.forward = forward;
    this.sideways = sideways;
    this.heading = headingDegrees;
  }

  public boolean atTarget() {
    return forward.atGoal() && sideways.atGoal() && heading.atGoal();
  }

  public ChassisSpeeds calculate(Pose2d current, Pose2d target) {
    double forwardSpeed = this.forward.calculate(current.getX(), target.getX());
    if (this.forward.atGoal()) {
      forwardSpeed = 0;
    }
    double sidewaysSpeed = this.sideways.calculate(current.getY(), target.getY());
    if (this.sideways.atGoal()) {
      sidewaysSpeed = 0;
    }
    double rotationalSpeed =
        this.heading.calculate(
            current.getRotation().getDegrees(), target.getRotation().getDegrees());
    if (this.heading.atGoal()) {
      rotationalSpeed = 0;
    }
    return new ChassisSpeeds(forwardSpeed, sidewaysSpeed, Math.toRadians(rotationalSpeed));
  }

  public void reset(Pose2d current) {
    forward.reset(current.getX());
    sideways.reset(current.getY());
    heading.reset(current.getRotation().getDegrees());
  }
}
