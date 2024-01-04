package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveRelativeCommand extends CommandBase {

    private Pose2d targetPose;
    private Translation2d limitTranslation;
    private HolonomicDriveController holonomicDriveController;

    private SwerveDrive swerve;

    public MoveRelativeCommand(Pose2d relativePose) {
        holonomicDriveController = new HolonomicDriveController(
            new PIDController(2, 0, 0), 
            new PIDController(2, 0, 0), 
            new ProfiledPIDController(1, 0, 0, new Constraints(4, 4))
        );
        swerve = SwerveDrive.getInstance();
        this.targetPose = new Pose2d(
            swerve.getEstimatorPose().getTranslation().plus(
                relativePose.getTranslation()
            ),
            swerve.getEstimatorPose().getRotation().plus(
                relativePose.getRotation()
            )
        );
        addRequirements(swerve);
    }

    public MoveRelativeCommand(Pose2d relativePose, Translation2d limitTranslation) {
        this(relativePose);
        this.limitTranslation = swerve.getEstimatorPose().getTranslation().plus(limitTranslation);
    }

    @Override
    public void execute() {

        ChassisSpeeds speeds = holonomicDriveController.calculate(swerve.getEstimatorPose(), targetPose, 2, targetPose.getRotation());
        Logger.getInstance().recordOutput("MRC/speeds", speeds.vxMetersPerSecond);
        swerve.driveRobotCentric(speeds);

    }

    @Override
    public boolean isFinished() {
        if (holonomicDriveController.atReference()) return true;
        if (limitTranslation != null) {
            if (
                limitTranslation.getX()-swerve.getEstimatorPose().getX() < 0
                || limitTranslation.getY()-swerve.getEstimatorPose().getY() < 0
            ) {
                return true;
            }
        }
        return false;
    }
    
}
