package frc.robot.hardware.motors;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class PositionMotor extends Motor {
    private ElevatorFeedforward ff;
    public PositionMotor(
        DoubleConsumer positionSetter,
        DoubleConsumer voltageSetter,
        DoubleSupplier positionGetter,
        DoubleSupplier velocityGetter,
        FeedbackController fb,
        ElevatorFeedforward ff,
        Loggable motorInfo
    ) {
        super(positionSetter, voltageSetter, positionGetter, velocityGetter, fb, motorInfo);
        this.ff = ff;
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
        double fbVolts = fb.calculate(positionGetter.getAsDouble(), target);
        double ffVolts = 0;
        if (ff != null) {
            ffVolts = ff.getKg() + ff.getKs() * Math.signum(fbVolts);
        }
        voltageSetter.accept(fbVolts + ffVolts);
    }

    @Override
    public boolean atTarget() {
        if (useVoltage) {
            return true;
        }
        fb.calculate(getPosition(), target);
        return fb.atGoal();
    }

    public static PositionMotor fromTalonFX(
        int canID,
        Consumer<TalonFX> config,
        FeedbackController fb,
        ElevatorFeedforward ff
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
        return new PositionMotor(
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

    public static PositionMotor fromSparkMax(
        int canID,
        boolean brushed,
        Consumer<SparkMax> config,
        FeedbackController fb,
        ElevatorFeedforward ff
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
        return new PositionMotor(
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

    public static PositionMotor fromTalonSRX(
        int canID,
        double conversionFactor,
        Consumer<TalonSRX> config,
        FeedbackController fb,
        ElevatorFeedforward ff
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
        return new PositionMotor(
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

    public static PositionMotor fromRealisticSim(
        FeedbackController fb,
        ElevatorFeedforward ff
    ) {
        FeedforwardSim sim = FeedforwardSim.createElevator(ff.getKg(), ff.getKs(), ff.getKv(), ff.getKa(), new State());
        return new PositionMotor(
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

    public static PositionMotor fromIdealSim(
        FeedbackController fb
    ) {
        State currentState = new State();
        return new PositionMotor(
            position -> currentState.position = position, 
            voltage -> {
                State nextState = fb.getSetpoint();
                currentState.position = nextState.position;
                currentState.velocity = nextState.velocity;
            }, 
            () -> currentState.position, 
            () -> currentState.velocity,
            fb, 
            null, 
            name -> {}
        );
    }
}
