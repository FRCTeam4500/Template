package frc.robot.hardware.motors;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class VelocityMotor extends Motor {
    private SimpleMotorFeedforward ff;
    public VelocityMotor(
        DoubleConsumer positionSetter,
        DoubleConsumer voltageSetter,
        DoubleSupplier positionGetter,
        DoubleSupplier velocityGetter,
        FeedbackController fb,
        SimpleMotorFeedforward ff,
        Loggable motorInfo
    ) {
        this.positionSetter = positionSetter;
        this.voltageSetter = voltageSetter;
        this.positionGetter = positionGetter;
        this.velocityGetter = velocityGetter;
        this.fb = fb;
        this.ff = ff;
        this.motorInfo = motorInfo;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            voltageSetter.accept(0);
            return;
        }
        if (useVoltage) {
            voltageSetter.accept(target);
            return;
        }
        double fbVolts = fb.calculate(velocityGetter.getAsDouble(), target);
        double ffVolts = 0;
        if (ff != null) {
            ffVolts = ff.getKs() * Math.signum(target) + ff.getKv() * target;
        }
        voltageSetter.accept(fbVolts + ffVolts);
    }

    @Override
    public boolean atTarget() {
        if (useVoltage) {
            return true;
        }
        fb.calculate(getVelocity(), target);
        return fb.atGoal();
    }

    public static VelocityMotor fromTalonFX(
        int canID,
        Consumer<TalonFX> config,
        FeedbackController fb,
        SimpleMotorFeedforward ff
    ) {
        if (RobotBase.isSimulation()) {
            if (ff == null) {
                return fromIdealSim(fb);
            } else {
                return fromRealisticSim(fb, ff);
            }
        }
        TalonFX motor = new TalonFX(canID);
        config.accept(motor);
        return new VelocityMotor(
            motor::setPosition, 
            motor::setVoltage, 
            () -> motor.getPosition().getValueAsDouble(), 
            () -> motor.getVelocity().getValueAsDouble(), 
            fb, 
            ff, 
            name -> {
                HoundLog.log(name + "/Temperature", motor.getDeviceTemp().getValueAsDouble());
                HoundLog.log(name + "/Stator Current", motor.getStatorCurrent().getValueAsDouble());
                HoundLog.log(name + "/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
                HoundLog.log(name + "/Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
            }
        );
    }

    public static VelocityMotor fromSparkMax(
        int canID,
        boolean brushed,
        Consumer<SparkMax> config,
        FeedbackController fb,
        SimpleMotorFeedforward ff
    ) {
        if (RobotBase.isSimulation()) {
            if (ff == null) {
                return fromIdealSim(fb);
            } else {
                return fromRealisticSim(fb, ff);
            }
        }
        SparkMax motor = new SparkMax(canID, brushed ? MotorType.kBrushed : MotorType.kBrushless);
        config.accept(motor);
        return new VelocityMotor(
            position -> motor.getEncoder().setPosition(position), 
            motor::setVoltage, 
            () -> motor.getEncoder().getPosition(), 
            () -> motor.getEncoder().getVelocity(), 
            fb, 
            ff, 
            name -> {
                HoundLog.log(name + "/Applied Volts", motor.getAppliedOutput() * motor.getBusVoltage());
                HoundLog.log(name + "/Temperature", motor.getMotorTemperature());
                HoundLog.log(name + "/Stator Current", motor.getOutputCurrent());
            }
        );
    }

    public static VelocityMotor fromTalonSRX(
        int canID,
        double conversionFactor,
        Consumer<TalonSRX> config,
        FeedbackController fb,
        SimpleMotorFeedforward ff
    ) {
        if (RobotBase.isSimulation()) {
            if (ff == null) {
                return fromIdealSim(fb);
            } else {
                return fromRealisticSim(fb, ff);
            }
        }
        TalonSRX motor = new TalonSRX(canID);
        config.accept(motor);
        return new VelocityMotor(
            position -> motor.setSelectedSensorPosition(position / conversionFactor), 
            voltage -> motor.set(ControlMode.PercentOutput, voltage / motor.getBusVoltage()), 
            () -> motor.getSelectedSensorPosition() * conversionFactor, 
            () -> motor.getSelectedSensorVelocity() * 10 * conversionFactor, 
            fb, 
            ff, 
            name -> {
                HoundLog.log(name + "/Bus Voltage", motor.getBusVoltage());
            }
        );
    }

    public static VelocityMotor fromRealisticSim(
        FeedbackController fb,
        SimpleMotorFeedforward ff
    ) {
        FeedforwardSim sim = FeedforwardSim.createFlywheel(ff.getKs(), ff.getKv(), ff.getKa(), new State());
        return new VelocityMotor(
            sim::resetPosition, 
            sim::setVoltage, 
            sim::getPosition, 
            sim::getVelocity, 
            fb, 
            ff, 
            name -> {
                HoundLog.log(name + "/Voltage", sim.getVoltage());
            }
        );
    }

    public static VelocityMotor fromIdealSim(
        FeedbackController fb
    ) {
        State currentState = new State();
        double[] positionHolder = new double[] {0};
        return new VelocityMotor(
            position -> positionHolder[0] = position, 
            voltage -> {
                if (voltage == 0) {
                    currentState.position = 0;
                    currentState.velocity = 0;
                } else {
                    State nextState = fb.getSetpoint();
                    currentState.position = nextState.position;
                    currentState.velocity = nextState.velocity;
                }
                positionHolder[0] += 0.02 * currentState.position;
                
            }, 
            () -> positionHolder[0], 
            () -> currentState.position, 
            fb, 
            null, 
            name -> {}
        );
    }
}
