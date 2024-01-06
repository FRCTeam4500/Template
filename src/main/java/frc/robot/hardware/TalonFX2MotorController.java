package frc.robot.hardware;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public class TalonFX2MotorController extends TalonFX implements EncodedMotorController {
    // private double TICKS_PER_RADIAN = 2048 / Math.PI / 2;
    private TalonFXConfiguration config;

    public TalonFX2MotorController(int deviceID) {
        super(deviceID);
        super.getConfigurator().apply(config);
        config = new TalonFXConfiguration();
        config.MotionMagic.MotionMagicAcceleration = 9999;
        config.MotionMagic.MotionMagicCruiseVelocity = 9999;
        // config_IntegralZone(0, 0);
        // configMotionCruiseVelocity(10000);
        // configMotionAcceleration(10000);
        // configAllowableClosedloopError(0, 0);
        // configClearPositionOnQuadIdx(true, 10);
    }

    @Override
    public void setOutput(double output) {
        // set(ControlMode.PercentOutput, output);
        setControl(new DutyCycleOut(output));
    }

    @Override
    public double getOutput() {
        // return getMotorOutputPercent();
        return get();
    }

    @Override
    public void setAngularVelocity(Rotation2d velocity) {
        // set(ControlMode.Velocity, velocity.getRadians() * TICKS_PER_RADIAN / 10.0);
        setControl(new VelocityDutyCycle(velocity.getRadians()));
    }

    @Override
    public Rotation2d getAngularVelocity() {
        // return Rotation2d.fromRadians(getSelectedSensorVelocity() / TICKS_PER_RADIAN * 10); 
        return Rotation2d.fromRotations(getRotorVelocity().getValueAsDouble());
    }

    @Override
    public void setAngle(Rotation2d angle) {
        setControl(new MotionMagicDutyCycle(angle.getRotations()));
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(getRotorPosition().getValueAsDouble());
    }

    @Override
    public boolean hasContinuousRotation() {
        return false;
    }

    @Override
    public TalonFX2MotorController configCurrentLimit(int currentLimit) {
        // configSupplyCurrentLimit(
        //     new SupplyCurrentLimitConfiguration(
        //         true, 
        //         currentLimit, 
        //         currentLimit + 1, 
        //         0.1
        //     ), 
        //     50
        // );
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.CurrentLimits.SupplyCurrentThreshold = currentLimit+1;
        config.CurrentLimits.SupplyTimeThreshold = 0.1;
        
        return this;
    }

    @Override
    public TalonFX2MotorController configPID(PIDConstants pid) {
        config.Slot0.kP = pid.kP;
        config.Slot0.kI =  pid.kI;
        config.Slot0.kD = pid.kD;
        return this;
    }

    @Override
    public TalonFX2MotorController configMinAngle(Rotation2d min) {
        // configReverseSoftLimitEnable(true);
        // configReverseSoftLimitThreshold(min.getRadians() * TICKS_PER_RADIAN);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min.getRotations();
        return this;
    }

    @Override
    public TalonFX2MotorController configMaxAngle(Rotation2d max) {
        // configForwardSoftLimitEnable(true);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max.getRotations();
        // configForwardSoftLimitThreshold(max.getRadians() * TICKS_PER_RADIAN);
        return this;
    }

    @Override
    public TalonFX2MotorController configMinOutput(double minOutput) {
        // configPeakOutputReverse(minOutput);
        config.MotorOutput.PeakReverseDutyCycle = minOutput;
       return this;
    }

    @Override
    public TalonFX2MotorController configMaxOutput(double maxOutput) {
        // configPeakOutputForward(maxOutput);
        config.MotorOutput.PeakForwardDutyCycle = maxOutput;
        return this;
    }
    
    @Override
    public TalonFX2MotorController configInverted(boolean shouldInvert) {
        setInverted(shouldInvert);
        return this;
    }

    @Override
    public TalonFX2MotorController configBrakeOnIdle(boolean shouldBreak) {
        // setNeutralMode(
        //     shouldBreak
        //     ? NeutralMode.Brake
        //     : NeutralMode.Coast
        // );
        if (shouldBreak) {
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        return this;
    }
}