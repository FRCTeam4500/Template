package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveRelativeCommand extends CommandBase {

    private Pose2d targetPose;
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
            swerve.getOdometryPose().getTranslation().plus(
                relativePose.getTranslation()
            ),
            swerve.getOdometryPose().getRotation().plus(
                relativePose.getRotation()
            )
        );
        addRequirements(swerve);
    }

    @Override
    public void execute() {

        ChassisSpeeds speeds = holonomicDriveController.calculate(swerve.getOdometryPose(), targetPose, 2, targetPose.getRotation());
        
        Logger.getInstance().recordOutput("MRC/speeds", speeds.vxMetersPerSecond);
        
        swerve.driveRobotCentric(speeds);

    }

    @Override
    public boolean isFinished() {
        if (holonomicDriveController.atReference()) return true;
        return false;
    }
    
}
