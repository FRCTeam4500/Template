package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.utilities.logging.HoundLog;
import java.util.function.Supplier;

public class StopTilting {
  private static ChassisSpeeds previousSpeed = new ChassisSpeeds();
  private static SwerveDriveKinematics kinematics;
  private static Supplier<Pose2d> pose;
  private static Transform3d baseCOM;
  private static Transform3d[] COMs;
  private static double baseMass;
  private static double[] masses;

  private static double forward;
  private static double backward;
  private static double left;
  private static double right;

  private static final double g = 9.81;

  public static void setupKinematics(SwerveDriveKinematics kinematics) {
    StopTilting.kinematics = kinematics;
  }

  public static void setupBase(Supplier<Pose2d> pose, Transform3d baseCOM, double baseMass) {
    StopTilting.pose = pose;
    StopTilting.baseCOM = baseCOM;
    StopTilting.baseMass = baseMass;
  }

  public static void setupSuperstructure(Transform3d[] componentCOMs, double[] componentMasses) {
    StopTilting.COMs = componentCOMs;
    StopTilting.masses = componentMasses;
  }

  public static void updateCenterOfMass(Transform3d[] componentTransforms) {
    double x = baseCOM.getX() * baseMass;
    double y = baseCOM.getY() * baseMass;
    double z = baseCOM.getZ() * baseMass;
    double mass = baseMass;
    for (int i = 0; i < componentTransforms.length; i++) {
      Transform3d componentCOM = componentTransforms[i].plus(COMs[i]);
      x += componentCOM.getX() * masses[i];
      y += componentCOM.getY() * masses[i];
      z += componentCOM.getZ() * masses[i];
      mass += masses[i];
    }
    Transform3d totalCOM = new Transform3d(x / mass, y / mass, z / mass, Rotation3d.kZero);

    HoundLog.log("COM", "COM", new Pose3d(pose.get()).plus(totalCOM));
    double number = 0;
    for (Translation2d tl : kinematics.getModules()) {
      if (tl.getX() > number) {
        number = tl.getX();
      }
    }
    forward = (g / totalCOM.getZ()) * (number - totalCOM.getX());
    number = 0;
    for (Translation2d tl : kinematics.getModules()) {
      if (tl.getX() < number) {
        number = tl.getX();
      }
    }
    backward = (g / totalCOM.getZ()) * (number - totalCOM.getX());
    number = 0;
    for (Translation2d tl : kinematics.getModules()) {
      if (tl.getY() > number) {
        number = tl.getY();
      }
    }
    left = (g / totalCOM.getZ()) * (number - totalCOM.getY());
    number = 0;
    for (Translation2d tl : kinematics.getModules()) {
      if (tl.getY() < number) {
        number = tl.getY();
      }
    }
    right = (g / totalCOM.getZ()) * (number - totalCOM.getY());
  }

  public static ChassisSpeeds limitAccel(ChassisSpeeds target) {
    ChassisSpeeds accel =
        new ChassisSpeeds(
            (target.vxMetersPerSecond - previousSpeed.vxMetersPerSecond) / 0.02,
            (target.vyMetersPerSecond - previousSpeed.vyMetersPerSecond) / 0.02,
            (target.omegaRadiansPerSecond - previousSpeed.omegaRadiansPerSecond) / 0.02);
    double forwardAccel = 0;
    double sidewaysAccel = 0;
    Translation2d accel2d = new Translation2d(accel.vxMetersPerSecond, accel.vyMetersPerSecond);
    if (accel2d.getX() > 0) {
      forwardAccel = Math.min(forward, accel2d.getX());
    } else {
      forwardAccel = Math.max(backward, accel2d.getX());
    }
    if (accel2d.getY() > 0) {
      sidewaysAccel = Math.min(left, accel2d.getY());
    } else {
      sidewaysAccel = Math.max(right, accel2d.getY());
    }
    previousSpeed =
        new ChassisSpeeds(
            forwardAccel * 0.02 + previousSpeed.vxMetersPerSecond,
            sidewaysAccel * 0.02 + previousSpeed.vyMetersPerSecond,
            target.omegaRadiansPerSecond);
    return previousSpeed;
  }
}
