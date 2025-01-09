package frc.robot.utilities.gamepieces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class ScoringLocations {
  //Blue
  public static final Pose3d ID17_L4_LEFT = new Pose3d(3.96, 3.43, 1.75, new Rotation3d(0, Math.PI / 2, 0));
  public static final Pose3d ID17_L4_RIGHT = new Pose3d(4.24, 3.27, 1.75, new Rotation3d(0, Math.PI / 2, 0));
  public static final Pose3d ID18_L4_LEFT = new Pose3d(3.7, 4.19, 1.75, new Rotation3d(0, Math.PI / 2, 0));
  public static final Pose3d ID18_L4_RIGHT = new Pose3d(3.7, 3.86, 1.75, new Rotation3d(0, Math.PI / 2, 0));
  public static final Pose3d ID19_L4_LEFT = new Pose3d(4.24, 4.78, 1.75, new Rotation3d(0, Math.PI / 2, 0));
  public static final Pose3d ID19_L4_RIGHT = new Pose3d(3.96, 4.62, 1.75, new Rotation3d(0, Math.PI / 2, 0));

  public static Pose3d flip(Pose3d og) {
    Rotation3d rot = og.getRotation();
    return new Pose3d(17.55 - og.getX(), 8.05 - og.getY(), og.getZ(), new Rotation3d(rot.getX(), rot.getY(), Math.PI - rot.getZ()));
  }
}
