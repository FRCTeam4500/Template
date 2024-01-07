package frc.robot.hardware;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public class TalonFXMotorController extends TalonFX implements EncodedMotorController {
    // private double TICKS_PER_RADIAN = 2048 / Math.PI / 2;
    private TalonFXConfiguration config;

    public TalonFXMotorController(int deviceID) {
        super(deviceID);
        config = new TalonFXConfiguration();
        getConfigurator().apply(config);
        config.MotionMagic.MotionMagicAcceleration = 9999;
        config.MotionMagic.MotionMagicCruiseVelocity = 9999;
    }

    @Override
    public void setOutput(double output) {
        setControl(new DutyCycleOut(output));
    }

    @Override
    public double getOutput() {
        return get();
    }

    @Override
    public void setAngularVelocity(Rotation2d velocity) {
        setControl(new VelocityDutyCycle(velocity.getRotations()));
    }

    @Override
    public Rotation2d getAngularVelocity() {
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
    public TalonFXMotorController configCurrentLimit(int currentLimit) {
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.CurrentLimits.SupplyCurrentThreshold = currentLimit+1;
        config.CurrentLimits.SupplyTimeThreshold = 0.1;
        return this;
    }

    @Override
    public TalonFXMotorController configPID(PIDConstants pid) {
        config.Slot0.kP = pid.kP;
        config.Slot0.kI =  pid.kI;
        config.Slot0.kD = pid.kD;
        return this;
    }

    @Override
    public TalonFXMotorController configMinAngle(Rotation2d min) {
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min.getRotations();
        return this;
    }

    @Override
    public TalonFXMotorController configMaxAngle(Rotation2d max) {
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max.getRotations();
        return this;
    }

    @Override
    public TalonFXMotorController configMinOutput(double minOutput) {
        config.MotorOutput.PeakReverseDutyCycle = minOutput;
       return this;
    }

    @Override
    public TalonFXMotorController configMaxOutput(double maxOutput) {
        config.MotorOutput.PeakForwardDutyCycle = maxOutput;
        return this;
    }
    
    @Override
    public TalonFXMotorController configInverted(boolean shouldInvert) {
        getConfigurator().refresh(config);
        if (shouldInvert) {
            if (config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive) {
                config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            } else {
                config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            }
        }
        return this;
    }

    @Override
    public TalonFXMotorController configBrakeOnIdle(boolean shouldBreak) {
        if (shouldBreak) {
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        return this;
    }
}