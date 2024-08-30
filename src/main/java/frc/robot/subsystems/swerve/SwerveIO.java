package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.base.SwerveBaseIO;
import frc.robot.subsystems.swerve.base.SwerveBaseReal;
import frc.robot.subsystems.swerve.base.SwerveBaseSim;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import dev.doglog.DogLog;

import java.util.function.Function;

public class SwerveIO extends SubsystemBase {
    private static SwerveIO instance;
    public static synchronized SwerveIO getInstance() {
        if (instance == null) instance = new SwerveIO();
        return instance;
    }
    
    private SwerveBaseIO base;
    private Rotation2d targetAngle;
    private PIDController anglePID;
    private SwerveIO() {
        base = RobotBase.isReal() ? new SwerveBaseReal() : new SwerveBaseSim();
        targetAngle = new Rotation2d();
        anglePID = new PIDController(5, 0, 0);
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
		anglePID.setTolerance(Math.PI / 32, Math.PI / 32);
		anglePID.setSetpoint(0);
        AutoBuilder.configureHolonomic(
            base::getPose, 
            base::setPose, 
            base::getSpeeds, 
            base::setSpeeds, 
            new HolonomicPathFollowerConfig(MAX_MODULE_SPEED, 0.39878808909, new ReplanningConfig(true, true)), 
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, 
            this
        );
    }

    @Override
    public void periodic() {
        base.periodic();
    }

    public void log() {
        DogLog.log("Target Angle", targetAngle);
        DogLog.log("Speeds", base.getSpeeds());
        DogLog.log("Pose", base.getPose());
        DogLog.log("Module States", base.getStates());
    }

    public Command fieldCentric(CommandXboxController xbox, Function<ChassisSpeeds, ChassisSpeeds> conversion) {
        return run(() -> {
            double speedCoefficient = Math.max(1 - xbox.getLeftTriggerAxis(), MIN_COEFFICIENT);
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            double forward = 0;
            double sideways = 0;
            if (alliance == Alliance.Blue) {
                forward = speedCoefficient * -xbox.getLeftY() * MAX_FORWARD_SPEED;
                sideways = speedCoefficient * -xbox.getLeftX() * MAX_SIDEWAYS_SPEED;
            } else {
                forward = speedCoefficient * xbox.getLeftY() * MAX_FORWARD_SPEED;
                sideways = speedCoefficient * xbox.getLeftX() * MAX_SIDEWAYS_SPEED;
            }
            double rotational = Math.toRadians(10 * speedCoefficient * -xbox.getRightX() * MAX_ROTATIONAL_SPEED);
            ChassisSpeeds original = new ChassisSpeeds(forward, sideways, rotational);
            base.setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                conversion.apply(original), 
                base.getPose().getRotation()
            ));
        });
    }

    public Command fieldCentric(CommandXboxController xbox) {
        return fieldCentric(xbox, speeds -> speeds);
    }

    public Command angleCentric(
        CommandXboxController xbox
    ) {
        return fieldCentric(
            xbox, 
            speeds -> { 
                targetAngle = Rotation2d.fromDegrees(
                    targetAngle.getDegrees() -
                    xbox.getRightX() * 
                    Math.max(MIN_COEFFICIENT, 1 - xbox.getLeftTriggerAxis()) *
                    MAX_ROTATIONAL_SPEED
                );
                return new ChassisSpeeds(
                    speeds.vxMetersPerSecond, 
                    speeds.vyMetersPerSecond,
                    calculateRotationalVelocityToTarget(targetAngle)
                );
        }).beforeStarting(
            () -> targetAngle = base.getPose().getRotation()
        ).withName("Angel Centric Drive");
    }

    public Command targetAngle(Rotation2d angle) {
        return Commands.runOnce(() -> targetAngle = angle);
    }

    public Command resetGyro() {
        return Commands.runOnce(() -> {
            targetAngle = DriverStation.getAlliance()
                .orElse(Alliance.Blue) == Alliance.Blue ? 
                new Rotation2d() : Rotation2d.fromDegrees(180);
            base.setAngle(targetAngle);
        });
    }

    public SwerveState getState() {
        return new SwerveState(base.getPose(), base.getSpeeds());
    }

    private double calculateRotationalVelocityToTarget(Rotation2d targetRotation) {
		double rotationalVelocity = anglePID.calculate(
			base.getPose().getRotation().getRadians(),
			targetRotation.getRadians()
		);
		if (anglePID.atSetpoint()) {
			rotationalVelocity = 0;
		}
		return rotationalVelocity;
	}

    public static record SwerveState(Pose2d pose, ChassisSpeeds speeds) {}
}
