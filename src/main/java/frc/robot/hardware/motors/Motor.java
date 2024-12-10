package frc.robot.hardware.motors;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.SysIDCommands;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

import static edu.wpi.first.units.Units.*;

public abstract class Motor extends SubsystemBase implements Loggable {
    protected boolean useVoltage = true;
    protected double target = 0;
    protected DoubleConsumer positionSetter;
    protected DoubleConsumer voltageSetter;
    protected DoubleSupplier positionGetter;
    protected DoubleSupplier velocityGetter;
    protected FeedbackController fb;
    protected Loggable motorInfo;

    public void setTarget(double target) {
        useVoltage = false;
        this.target = target;
    }

    public void setVoltage(double volts) {
        useVoltage = true;
        target = volts;
    }

    public void resetPosition(double actualPosition) {
        positionSetter.accept(actualPosition);
    }

    public double getPosition() {
        return positionGetter.getAsDouble();
    }

    public double getVelocity() {
        return velocityGetter.getAsDouble();
    }

    public SysIDCommands getSysIDCommands(
        String name, 
        double voltageRampRate, 
        double stepVoltage, 
        double timeout
    ) {
        Config config = new Config(
            Volts.of(voltageRampRate).per(Seconds), 
            Volts.of(stepVoltage), 
            Seconds.of(timeout)
        );
        Mechanism mech = new Mechanism(
            voltage -> setVoltage(voltage.in(Volts)), 
            log -> log.motor("Motor")
                .value("Position", getPosition(), "IDK")
                .value("Velocity", getVelocity(), "IDK")
                .value("Voltage", target, "Volts"),
            this,
            name
        );
        SysIdRoutine routine = new SysIdRoutine(config, mech);
        return new SysIDCommands(
            routine.dynamic(Direction.kForward), 
            routine.dynamic(Direction.kReverse), 
            routine.quasistatic(Direction.kForward), 
            routine.quasistatic(Direction.kReverse)
        );
    }

    public SysIDCommands getSynchronizedSysIDCommands(
        String name, 
        double voltageRampRate,
        double stepVoltage,
        double timeout,
        Motor... otherMotors
    ) {
        Config config = new Config(
            Volts.of(voltageRampRate).per(Seconds), 
            Volts.of(stepVoltage), 
            Seconds.of(timeout)
        );
        Mechanism mech = new Mechanism(
            voltage -> {
                setVoltage(voltage.in(Volts));
                for (Motor motor : otherMotors) {
                    motor.setVoltage(voltage.in(Volts));
                }
            }, 
            log -> {
                log.motor("Motor0")
                    .value("Position", getPosition(), "IDK")
                    .value("Velocity", getVelocity(), "IDK")
                    .value("Voltage", target, "Volts");
                for (int i = 0; i < otherMotors.length; i++) {
                    Motor motor = otherMotors[i];
                    log.motor("Motor" + (i + 1))
                        .value("Position", motor.getPosition(), "IDK")
                        .value("Velocity", motor.getVelocity(), "IDK")
                        .value("Voltage", motor.target, "Volts");
                }
            },
            this,
            name
        );
        SysIdRoutine routine = new SysIdRoutine(config, mech);
        SysIDCommands commands = new SysIDCommands(
            routine.dynamic(Direction.kForward), 
            routine.dynamic(Direction.kReverse), 
            routine.quasistatic(Direction.kForward), 
            routine.quasistatic(Direction.kReverse)
        );
        commands.dynamicForward().addRequirements(otherMotors);
        commands.dynamicReverse().addRequirements(otherMotors);
        commands.quasistaticForward().addRequirements(otherMotors);
        commands.quasistaticReverse().addRequirements(otherMotors);
        return commands;
    }

    @Override
    public void log(String name) {
        HoundLog.log(name + "/Motor Info", motorInfo);
        HoundLog.log(name + "/Voltage Control", useVoltage);
        HoundLog.log(name + "/Current Position", getPosition());
        HoundLog.log(name + "/Current Velocity", getVelocity());
        HoundLog.log(name + "/Target", target);
        HoundLog.log(name + "/At Target", atTarget());
    }

    public abstract boolean atTarget();

    @Override
    public abstract void periodic();
}
