package frc.robot.hardware;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.util.Units;

public class SparkMaxMotorController extends CANSparkMax implements EncodedMotorController {
	public SparkMaxMotorController(int deviceID, MotorType type) {
		super(deviceID, type);
	}

    @Override
	public double getAngle() {
		return Units.rotationsToRadians(getEncoder().getPosition());
	}

    @Override
	public void setAngle(double position) {
		getPIDController()
			.setReference(Units.radiansToRotations(position), ControlType.kPosition);
	}

    @Override
	public void setOutput(double output) {
		set(output);
	}

    @Override
	public double getOutput() {
		return get();
	}

    @Override
	public double getAngularVelocity() {
		return Units.rotationsPerMinuteToRadiansPerSecond(getEncoder().getVelocity());
	}
	
    @Override
	public void setAngularVelocity(double velocity) {
		getPIDController()
			.setReference(
				Units.radiansPerSecondToRotationsPerMinute(velocity),
				ControlType.kVelocity
			);
	}

	@Override
	public EncodedMotorController setCurrentLimit(int currentLimit) {
		setSmartCurrentLimit(currentLimit);
		return this;
	}

	@Override
	public EncodedMotorController setPID(PIDConstants pid) {
		SparkMaxPIDController controller = getPIDController();
		controller.setP(pid.kP);
		controller.setI(pid.kI);
		controller.setD(pid.kD);
		return this;
	}

	@Override
	public EncodedMotorController setMinAngle(double minPosition) {
		setSoftLimit(SoftLimitDirection.kReverse, (float) Units.radiansToRotations(minPosition));
		return this;
	}

	@Override
	public EncodedMotorController setMaxAngle(double maxPosition) {
		setSoftLimit(SoftLimitDirection.kForward, (float) Units.radiansToRotations(maxPosition));
		return this;
	}

	@Override
	public EncodedMotorController setMinOutput(double minOutput) {
		SparkMaxPIDController controller = getPIDController();
		controller.setOutputRange(minOutput, controller.getOutputMax());
		return this;
	}

	@Override
	public EncodedMotorController setMaxOutput(double maxOutput) {
		SparkMaxPIDController controller = getPIDController();
		controller.setOutputRange(controller.getOutputMin(), maxOutput);
		return this;
	}

	@Override
	public EncodedMotorController setInversion(boolean shouldInvert) {
		super.setInverted(shouldInvert);
		return this;
	}

	@Override
	public EncodedMotorController setBrakeOnIdle(boolean shouldBreak) {
        setIdleMode(
          shouldBreak
          ? IdleMode.kBrake
          : IdleMode.kCoast  
        );
		return this;
	}

	@Override
	public EncodedMotorController setAngleTolerance(double tolerance) {
		// Not possible on a spark max
		return this;
	}
}
