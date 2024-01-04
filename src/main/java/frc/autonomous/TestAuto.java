package frc.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.MoveRelativeCommand;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TestAuto extends SequentialCommandGroup {

    public TestAuto() {
        super(
            new InstantCommand(() -> SwerveDrive.getInstance().zeroRobotAngle()),
            new MoveRelativeCommand(new Pose2d(0.5,0, new Rotation2d()))
        );
    }
    
}
