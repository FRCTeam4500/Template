package frc.robot.utilities.gamepieces;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

public class Gamepiece {
    private Optional<Movement> movement;
    private Pose3d pose;

    public Gamepiece(Translation2d pose) {
        this(new Pose3d(new Translation3d(pose), new Rotation3d()));
    }

    public Gamepiece(Pose3d pose) {
        this.pose = pose;
        this.movement = Optional.empty();
    }

    public void setPose(Pose3d pose) {
        this.pose = pose;
    }

    public void setPose(Translation2d pose) {
        setPose(new Pose3d(new Translation3d(pose), new Rotation3d()));
    }

    protected boolean shouldDelete() {
        return movement.isPresent() && movement.get().toDelete && movement.get().done();
    }

    public Pose3d getPose() {
        if (!movement.isPresent() || movement.get().done()) {
            clearMovement();
            return pose;
        }
        pose = movement.get().getPose();
        return pose;
    }

    public void setMovement(Pose3d endPose, double duration, boolean toDelete) {
        double time = Timer.getFPGATimestamp();
        setMovement(endPose, time, time + duration, toDelete);
    }

    public void setMovement(Pose3d endPose, double startTime, double endTime, boolean toDelete) {
        movement = Optional.of(new Movement(pose, endPose, startTime, endTime, toDelete));
    }

    public void clearMovement() {
        movement = Optional.empty();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Gamepiece piece) {
            return piece.movement.equals(this.movement)
            && piece.pose.equals(this.pose);
        }
        return false;
    }
}
