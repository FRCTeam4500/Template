package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.base.SwerveBaseIO;
import frc.robot.subsystems.swerve.base.SwerveBaseReal;
import frc.robot.subsystems.swerve.base.SwerveBaseSim;
import frc.robot.utilities.EZLogger.LogAccess;
import frc.robot.utilities.EZLogger.Loggable;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

public class SwerveIO extends SubsystemBase implements Loggable {
    private static SwerveIO instance;
    public static synchronized SwerveIO getInstance() {
        if (instance == null) instance = new SwerveIO();
        return instance;
    }
    
    private SwerveBaseIO base;
    private Rotation2d targetAngle;
    private PIDController anglePID;
    private Field2d field;
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
        field = new Field2d();
    }

    @Override
    public void periodic() {
        base.periodic();
        field.setRobotPose(base.getPose());
    }

    @Override
    public void log(LogAccess table) {
        table.put("Target Angle", targetAngle);
        table.put("Speeds", base.getSpeeds());
        table.put("Pose", base.getPose());
        table.put("Module States", base.getStates());
        table.put("Field", field);
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
        CommandXboxController xbox,
        BooleanSupplier faceForward,
        BooleanSupplier faceBackwards,
        BooleanSupplier faceBlueRight,
        BooleanSupplier faceBlueLeft
    ) {
        return fieldCentric(
            xbox, 
            speeds -> { 
                targetAngle = calculateTargetAngle(
                    xbox,
                    faceForward,
                    faceBackwards,
                    faceBlueRight,
                    faceBlueLeft
                );
                return new ChassisSpeeds(
                    speeds.vxMetersPerSecond, 
                    speeds.vyMetersPerSecond,
                    calculateRotationalVelocityToTarget(targetAngle)
                );
        }).beforeStarting(() -> targetAngle = base.getPose().getRotation());
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

    private Rotation2d calculateTargetAngle(
        CommandXboxController xbox,
        BooleanSupplier faceForward,
        BooleanSupplier faceBackwards,
        BooleanSupplier faceBlueRight,
        BooleanSupplier faceBlueLeft
    ) {
        double allianceCoefficient = 
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ?
                1 : -1;
        if (faceForward.getAsBoolean())
            return Rotation2d.fromDegrees(90 - allianceCoefficient * 90);
        else if (faceBackwards.getAsBoolean()) {
            return Rotation2d.fromDegrees(90 + allianceCoefficient * 90);
        }
        else if (faceBlueRight.getAsBoolean()) 
            return Rotation2d.fromDegrees(-90);
        else if (faceBlueLeft.getAsBoolean())
            return Rotation2d.fromDegrees(90);
        else
            return Rotation2d.fromDegrees(
                targetAngle.getDegrees() -
                xbox.getRightX() * Math.max(MIN_COEFFICIENT, 1 - xbox.getLeftTriggerAxis()) * MAX_ROTATIONAL_SPEED
            );
    } 

    public static record SwerveState(Pose2d pose, ChassisSpeeds speeds) {}
}
