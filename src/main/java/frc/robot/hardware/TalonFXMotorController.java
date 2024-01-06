package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public class TalonFXMotorController extends TalonFX implements EncodedMotorController {
    private double TICKS_PER_RADIAN = 2048 / Math.PI / 2;

    public TalonFXMotorController(int deviceID) {
        super(deviceID);
        config_IntegralZone(0, 0);
        configMotionCruiseVelocity(10000);
        configMotionAcceleration(10000);
        configAllowableClosedloopError(0, 0);
        configClearPositionOnQuadIdx(true, 10);
    }

    @Override
    public void setOutput(double output) {
        set(ControlMode.PercentOutput, output);
    }

    @Override
    public double getOutput() {
        return getMotorOutputPercent();
    }

    @Override
    public void setAngularVelocity(Rotation2d velocity) {
        set(ControlMode.Velocity, velocity.getRadians() * TICKS_PER_RADIAN / 10.0);
    }

    @Override
    public Rotation2d getAngularVelocity() {
        return Rotation2d.fromRadians(getSelectedSensorVelocity() / TICKS_PER_RADIAN * 10);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        set(ControlMode.MotionMagic, angle.getRadians() * TICKS_PER_RADIAN);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(getSelectedSensorPosition() / TICKS_PER_RADIAN);
    }

    @Override
    public boolean hasContinuousRotation() {
        return false;
    }

    @Override
    public TalonFXMotorController configCurrentLimit(int currentLimit) {
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
    public TalonFXMotorController configPID(PIDConstants pid) {
        config_kP(0, pid.kP);
        config_kI(0, pid.kI);
        config_kD(0, pid.kD);
        return this;
    }

    @Override
    public TalonFXMotorController configMinAngle(Rotation2d min) {
        configReverseSoftLimitEnable(true);
        configReverseSoftLimitThreshold(min.getRadians() * TICKS_PER_RADIAN);
        return this;
    }

    @Override
    public TalonFXMotorController configMaxAngle(Rotation2d max) {
        configForwardSoftLimitEnable(true);
        configForwardSoftLimitThreshold(max.getRadians() * TICKS_PER_RADIAN);
        return this;
    }

    @Override
    public TalonFXMotorController configMinOutput(double minOutput) {
        configPeakOutputReverse(minOutput);
       return this;
    }

    @Override
    public TalonFXMotorController configMaxOutput(double maxOutput) {
        configPeakOutputForward(maxOutput);
        return this;
    }
    
    @Override
    public TalonFXMotorController configInverted(boolean shouldInvert) {
        setInverted(shouldInvert);
        return this;
    }

    @Override
    public TalonFXMotorController configBrakeOnIdle(boolean shouldBreak) {
        setNeutralMode(
            shouldBreak
            ? NeutralMode.Brake
            : NeutralMode.Coast
        );
        return this;
    }
}