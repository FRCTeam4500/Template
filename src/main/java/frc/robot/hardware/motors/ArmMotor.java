package frc.robot.hardware.motors;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class ArmMotor extends SubsystemBase implements Loggable {
    private double target;
    private DoubleConsumer positionSetter;
    private DoubleConsumer voltageSetter;
    private DoubleSupplier positionGetter;
    private FeedbackController fb;
    private ArmFeedforward ff;
    private Loggable motorInfo;

    public ArmMotor(
        DoubleConsumer positionSetter,
        DoubleConsumer voltageSetter,
        DoubleSupplier positionGetter,
        FeedbackController fb,
        ArmFeedforward ff,
        Loggable motorInfo
    ) {
        target = Double.MAX_VALUE;
        this.positionSetter = positionSetter;
        this.voltageSetter = voltageSetter;
        this.positionGetter = positionGetter;
        this.fb = fb;
        this.ff = ff;
        this.motorInfo = motorInfo;
    }

    public void setTarget(double rotations) {
        target = rotations;
    }

    public void resetPosition(double rotations) {
        positionSetter.accept(rotations);
    }

    public void coast() {
        target = Double.MAX_VALUE;
    }

    public double getRotations() {
        return positionGetter.getAsDouble();
    }

    public boolean atTarget() {
        if (target == Double.MAX_VALUE) {
            return true;
        }
        fb.calculate(getRotations(), target);
        return fb.atGoal();
    }

    @Override
    public void log(String name) {
        motorInfo.log(name + "/Motor Info");
        HoundLog.log(name + "/Coasting", target == Double.MAX_VALUE);
        HoundLog.log(name + "/Current Rotation", getRotations());
        if (target != Double.MAX_VALUE) {
            HoundLog.log(name + "/Target Rotation", target);
        }
        HoundLog.log(name + "/At Target", atTarget());
    }

    @Override
    public void periodic() {
        if (target == Double.MAX_VALUE || DriverStation.isDisabled()) {
            voltageSetter.accept(0);
            return;
        }
        double fbVolts = fb.calculate(getRotations(), target);
        double ffVolts = 0;
        if (ff != null) {
            ffVolts = ff.ks * Math.signum(fbVolts) 
                        + ff.kg * Math.cos(
                            getRotations() * Math.PI * 2
                        );
        }
        voltageSetter.accept(fbVolts + ffVolts);
    }

    public static ArmMotor fromTalonFX(
        int canID,
        Consumer<TalonFX> config,
        FeedbackController fb,
        ArmFeedforward ff
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
        return new ArmMotor(
            motor::setPosition, 
            motor::setVoltage, 
            () -> motor.getPosition().getValueAsDouble(), 
            fb, 
            ff, 
            name -> {
                HoundLog.log(name + "/Velocity", motor.getVelocity().getValueAsDouble());
                HoundLog.log(name + "/Temperature", motor.getDeviceTemp().getValueAsDouble());
                HoundLog.log(name + "/Stator Current", motor.getStatorCurrent().getValueAsDouble());
                HoundLog.log(name + "/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
                HoundLog.log(name + "/Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
            }
        );
    } 

    public static ArmMotor fromSparkMax(
        int canID,
        boolean brushed,
        Consumer<CANSparkMax> config,
        FeedbackController fb,
        ArmFeedforward ff
    ) {
        if (RobotBase.isSimulation()) {
            if (ff == null) {
                return fromIdealSim(fb);
            } else {
                return fromRealisticSim(fb, ff);
            }
        }
        CANSparkMax motor = new CANSparkMax(canID, brushed ? MotorType.kBrushed : MotorType.kBrushless); 
        config.accept(motor);
        return new ArmMotor(
            position -> motor.getEncoder().setPosition(position), 
            motor::setVoltage, 
            () -> motor.getEncoder().getPosition(), 
            fb, 
            ff, 
            name -> {
                HoundLog.log(name + "/Applied Volts", motor.getAppliedOutput() * motor.getBusVoltage());
                HoundLog.log(name + "/Temperature", motor.getMotorTemperature());
                HoundLog.log(name + "/Stator Current", motor.getOutputCurrent());
                HoundLog.log(name + "/Velocity", motor.getEncoder().getVelocity());
            }
        );
    }

    public static ArmMotor fromTalonSRX(
        int canID,
        double conversionFactor,
        Consumer<TalonSRX> config,
        FeedbackController fb,
        ArmFeedforward ff
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
        return new ArmMotor(
            position -> motor.setSelectedSensorPosition(position / conversionFactor), 
            voltage -> motor.set(ControlMode.PercentOutput, voltage / motor.getBusVoltage()), 
            motor::getSelectedSensorPosition, 
            fb, 
            ff, 
            name -> {
                HoundLog.log(name + "/Velocity", motor.getSelectedSensorVelocity() * 10 * conversionFactor);
                HoundLog.log(name + "/Bus Voltage", motor.getBusVoltage());
            }
        );
    }

    public static ArmMotor fromRealisticSim(
        FeedbackController fb,
        ArmFeedforward ff
    ) {
        FeedforwardSim sim = FeedforwardSim.createElevator(ff.kg, ff.ks, ff.kv, ff.ka, new State());
        return new ArmMotor(
            sim::resetPosition, 
            sim::setVoltage, 
            sim::getPosition, 
            fb, 
            ff, 
            name -> {
                HoundLog.log(name + "/Voltage", sim.getVoltage());
                HoundLog.log(name + "/Velocity", sim.getVelocity());
            }
        );
    }

    public static ArmMotor fromIdealSim(
        FeedbackController fb
    ) {
        State currentState = new State();
        return new ArmMotor(
            position -> currentState.position = position, 
            voltage -> {
                State nextState = fb.getSetpoint();
                currentState.position = nextState.position;
                currentState.velocity = nextState.velocity;
            }, 
            () -> currentState.position, 
            fb, 
            null, 
            name -> {
                HoundLog.log(name + "/Velocity", currentState.velocity);
            }
        );
    }
}
