package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.pathplanner.lib.util.PIDConstants;

public class TalonSRXMotorController extends TalonSRX implements EncodedMotorController {
    private double TICKS_PER_RADIAN = 4096 / Math.PI / 2;

    public TalonSRXMotorController(int deviceID) {
        super(deviceID);
    }

    @Override
    public void setOutput(double targetPercentOutput) {
        set(ControlMode.PercentOutput, targetPercentOutput);
    }

    @Override
    public double getPercentOutput() {
        return getMotorOutputPercent();
    }

    @Override
    public void setAngularVelocity(double targetAngularVelocity) {
        set(ControlMode.Velocity, targetAngularVelocity * TICKS_PER_RADIAN / 10.0);
    }

    @Override
    public double getAngularVelocity() {
        return getSelectedSensorVelocity() / TICKS_PER_RADIAN * 10;
    }

    @Override
    public void setAngle(double targetAngle) {
        set(ControlMode.Position, targetAngle * TICKS_PER_RADIAN);
    }

    @Override
    public double getAngleRadians() {
        return getSelectedSensorPosition() / TICKS_PER_RADIAN;
    }

    @Override
    public boolean hasContinuousRotation() {
        return true;
    }

    @Override
    public EncodedMotorController setCurrentLimit(int currentLimit) {
        configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(
                true, 
                currentLimit, 
                currentLimit + 1, 
                0.1
            ), 
            50
        );
        return this;
    }

    @Override
    public EncodedMotorController setPID(PIDConstants pid) {
        config_kP(0, pid.kP);
        config_kI(0, pid.kI);
        config_kD(0, pid.kD);
        return this;
    }

    @Override
    public EncodedMotorController setMinAngle(double minPosition) {
        configReverseSoftLimitEnable(true);
        configReverseSoftLimitThreshold(minPosition * TICKS_PER_RADIAN);
        return this;
    }

    @Override
    public EncodedMotorController setMaxAngle(double maxPosition) {
        configForwardSoftLimitEnable(true);
        configForwardSoftLimitThreshold(maxPosition * TICKS_PER_RADIAN);
        return this;
    }

    @Override
    public EncodedMotorController setMinOutput(double minOutput) {
        configPeakOutputReverse(minOutput);
       return this;
    }

    @Override
    public EncodedMotorController setMaxOutput(double maxOutput) {
        configPeakOutputForward(maxOutput);
        return this;
    }
    
    @Override
    public EncodedMotorController setInversion(boolean shouldInvert) {
        setInverted(shouldInvert);
        return this;
    }

    @Override
    public EncodedMotorController setBrakeOnIdle(boolean shouldBreak) {
        setNeutralMode(
            shouldBreak
            ? NeutralMode.Brake
            : NeutralMode.Coast
        );
        return this;
    }

    @Override
    public EncodedMotorController setAngleTolerance(double tolerance) {
        configAllowableClosedloopError(0, tolerance * TICKS_PER_RADIAN);
        return this;
    }
    
}