package frc.robot.hardware;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import frc.robot.hardware.interfaces.SwerveMotorController;
import frc.robot.subsystems.swerve.SwerveModule.SwerveMotorConfig;

public class SparkMaxMotorController extends CANSparkMax implements SwerveMotorController {
	public SparkMaxMotorController(int deviceID, MotorType type) {
		super(deviceID, type);
	}

	public double getAngle() {
		return Units.radiansToRotations(getEncoder().getPosition());
	}

	public void setAngle(double position) {
		getPIDController()
			.setReference(Units.radiansToRotations(position), ControlType.kPosition);
	}

	public void setOutput(double output) {
		set(output);
	}

	public double getOutput() {
		return get();
	}

	public double getAngularVelocity() {
		return Units.rotationsPerMinuteToRadiansPerSecond(getEncoder().getVelocity());
	}
	
	public void setAngularVelocity(double velocity) {
		getPIDController()
			.setReference(
				Units.radiansPerSecondToRotationsPerMinute(velocity),
				ControlType.kVelocity
			);
	}

	public void configureForSwerve(SwerveMotorConfig config) {
		setInverted(config.invert);
        getPIDController().setP(config.pid.kP);
        getPIDController().setI(config.pid.kI);
        getPIDController().setD(config.pid.kD);
        getPIDController().setFF(0);
        setSmartCurrentLimit(config.currentLimit);
	}
}
