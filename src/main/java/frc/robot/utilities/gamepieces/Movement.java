package frc.robot.utilities.gamepieces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;

public class Movement {
    private Pose3d startPose;
    private Pose3d endPose;
    private double startTime;
    private double endTime;
    public final boolean toDelete;

    public Movement(Pose3d startPose, Pose3d endPose, double startTime, double endTime, boolean toDelete) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.startTime = startTime;
        this.endTime = endTime;
        this.toDelete = toDelete;
    }

    public Pose3d getPose() {
        return startPose.interpolate(
          endPose, (Timer.getFPGATimestamp() - startTime) / (endTime - startTime));
    }

    public boolean done() {
        return Timer.getFPGATimestamp() > endTime;
    }

    @Override
    public boolean equals(Object obj) {
      if (obj instanceof Movement movement) {
        return movement.startPose.equals(this.startPose)
            && movement.endPose.equals(this.endPose)
            && movement.startTime == this.startTime
            && movement.endTime == this.endTime;
      } else {
        return false;
      }
    }
}
