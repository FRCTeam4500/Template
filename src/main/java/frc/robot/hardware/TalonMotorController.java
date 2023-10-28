package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import frc.robot.Constants.EnumConstants.TalonModel;
import frc.robot.hardware.interfaces.SwerveMotorController;
import frc.robot.subsystems.swerve.SwerveModule.SwerveMotorConfig;

public class TalonMotorController extends BaseTalon implements SwerveMotorController{
    private TalonModel model;

    public TalonMotorController(int deviceID, TalonModel model) {
        super(deviceID, model.name);
        this.model = model;
    }

    public void setOutput(double targetPercentOutput) {
        set(ControlMode.PercentOutput, targetPercentOutput);
    }

    public double getOutput() {
        return getMotorOutputPercent();
    }

    public void setAngularVelocity(double targetAngularVelocity) {
        set(ControlMode.Velocity, targetAngularVelocity * model.ticksPerRadian / 10.0);
    }

    public double getAngularVelocity() {
        return getSelectedSensorVelocity() / model.ticksPerRadian * 10;
    }

    public void setAngle(double targetAngle) {
        if (model == TalonModel.TalonSRX) {
            set(ControlMode.Position, targetAngle * model.ticksPerRadian);
        } else {
            set(ControlMode.MotionMagic, targetAngle * model.ticksPerRadian);
        }
    }

    public double getAngle() {
        return getSelectedSensorPosition() / model.ticksPerRadian;
    }

    public void configureForSwerve(SwerveMotorConfig config){
        configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, config.currentLimit, config.currentLimit + 1, 0.1), 50);
        setInverted(config.invert);
        config_kP(0, config.pid.kP);
        config_kI(0, config.pid.kI);
        config_kD(0, config.pid.kD);
        config_IntegralZone(0, 0);
        configMotionCruiseVelocity(10000);
        configMotionAcceleration(10000);
        configAllowableClosedloopError(0, 0);
        configClearPositionOnQuadIdx(true, 10);
    }
}
