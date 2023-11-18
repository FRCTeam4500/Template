package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.pathplanner.lib.auto.PIDConstants;

public class TalonMotorController implements EncodedMotorController{
    private TalonModel model;
    private BaseTalon innerTalon;

    public TalonMotorController(int deviceID, TalonModel model) {
        switch (model) {
            case TalonFX:
                innerTalon = new TalonFX(deviceID);
                innerTalon.config_IntegralZone(0, 0);
                innerTalon.configMotionCruiseVelocity(10000);
                innerTalon.configMotionAcceleration(10000);
                innerTalon.configAllowableClosedloopError(0, 0);
                innerTalon.configClearPositionOnQuadIdx(true, 10);
                break;
            case TalonSRX:
                innerTalon = new TalonSRX(deviceID);
                break;
        }
        this.model = model;
    }

    public static enum TalonModel {
        TalonFX("Talon FX", 2048 / Math.PI / 2),
        TalonSRX("Talon SRX", 4096 / Math.PI / 2);
        
        public String name;
        public double ticksPerRadian;

        private TalonModel(String name, double ticksPerRadian) {
            this.name = name;
            this.ticksPerRadian = ticksPerRadian;
        }
    }

    @Override
    public void setOutput(double targetPercentOutput) {
        innerTalon.set(ControlMode.PercentOutput, targetPercentOutput);
    }

    @Override
    public double getOutput() {
        return innerTalon.getMotorOutputPercent();
    }

    @Override
    public void setAngularVelocity(double targetAngularVelocity) {
        innerTalon.set(ControlMode.Velocity, targetAngularVelocity * model.ticksPerRadian / 10.0);
    }

    @Override
    public double getAngularVelocity() {
        return innerTalon.getSelectedSensorVelocity() / model.ticksPerRadian * 10;
    }

    @Override
    public void setAngle(double targetAngle) {
        if (model == TalonModel.TalonSRX) {
            innerTalon.set(ControlMode.Position, targetAngle * model.ticksPerRadian);
        } else {
            innerTalon.set(ControlMode.MotionMagic, targetAngle * model.ticksPerRadian);
        }
    }

    @Override
    public double getAngle() {
        return innerTalon.getSelectedSensorPosition() / model.ticksPerRadian;
    }

    @Override
    public EncodedMotorController setCurrentLimit(int currentLimit) {
        innerTalon.configSupplyCurrentLimit(
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
        innerTalon.config_kP(0, pid.kP);
        innerTalon.config_kI(0, pid.kI);
        innerTalon.config_kD(0, pid.kD);
        return this;
    }

    @Override
    public EncodedMotorController setMinAngle(double minPosition) {
        innerTalon.configReverseSoftLimitEnable(true);
        innerTalon.configReverseSoftLimitThreshold(minPosition * model.ticksPerRadian);
        return this;
    }

    @Override
    public EncodedMotorController setMaxAngle(double maxPosition) {
        innerTalon.configForwardSoftLimitEnable(true);
        innerTalon.configForwardSoftLimitThreshold(maxPosition * model.ticksPerRadian);
        return this;
    }

    @Override
    public EncodedMotorController setMinOutput(double minOutput) {
        innerTalon.configPeakOutputReverse(minOutput);
       return this;
    }

    @Override
    public EncodedMotorController setMaxOutput(double maxOutput) {
        innerTalon.configPeakOutputForward(maxOutput);
        return this;
    }
    
    @Override
    public EncodedMotorController setInversion(boolean shouldInvert) {
        innerTalon.setInverted(shouldInvert);
        return this;
    }

    @Override
    public EncodedMotorController setBrakeOnIdle(boolean shouldBreak) {
        innerTalon.setNeutralMode(
            shouldBreak
            ? NeutralMode.Brake
            : NeutralMode.Coast
        );
        return this;
    }

    @Override
    public EncodedMotorController setAngleTolerance(double tolerance) {
        innerTalon.configAllowableClosedloopError(0, tolerance * model.ticksPerRadian);
        return this;
    }
}
