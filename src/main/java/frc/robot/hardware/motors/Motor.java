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

/**
 * A base class that represents a motor. Use {@link PositionMotor}, {@link VelocityMotor}, or {@link ArmMotor} 
 * for a specific implementaion.
 * 
 * @implNote The goal of the motor (position, velocity, etc) are left up to the implementor,
 * so long as {@link #setTarget} sets the correct goal type, and {@link #atTarget} returns true
 * when that goal has been reached
 */
public abstract class Motor extends SubsystemBase implements Loggable {
    protected boolean useVoltage = true;
    protected double target = 0;
    protected DoubleConsumer positionSetter = position -> {};
    protected DoubleConsumer voltageSetter = voltage -> {};
    protected DoubleSupplier positionGetter;
    protected DoubleSupplier velocityGetter;
    protected FeedbackController fb;
    protected Loggable motorInfo;

    /**
     * Creates a new motor where the given parameters are used to interface with the hardware or sim
     * @param positionSetter A function that sets the current position of the motor to the passed in value
     * @param voltageSetter A function that applies the passed in value as voltage to the motor
     * @param positionGetter A function that returns the current position of the motor
     * @param velocityGetter A function that returns the current velocity of the motor, in units per second
     * @param fb A feedback controller, which drives the motor to its goal
     * @param motorInfo A {@link Loggable} which logs information about the motor, such as applied voltage, tempreture, and current
     */
    public Motor(
        DoubleConsumer positionSetter,
        DoubleConsumer voltageSetter,
        DoubleSupplier positionGetter,
        DoubleSupplier velocityGetter,
        FeedbackController fb,
        Loggable motorInfo
    ) {
        this.positionSetter = positionSetter;
        this.voltageSetter = voltageSetter;
        this.positionGetter = positionGetter;
        this.velocityGetter = velocityGetter;
        this.fb = fb;
        this.motorInfo = motorInfo;
    }

    /**
     * Sets the target position/velocity for the motor.
     * @param target the target position/velocity to go to
     */
    public void setTarget(double target) {
        useVoltage = false;
        this.target = target;
    }

    /**
     * Sets the target voltage. 
     * @param volts the target voltage...
     * @apiNote Using this method causes {@link #atTarget()} to always return true!
     */
    public void setVoltage(double volts) {
        useVoltage = true;
        target = volts;
    }

    /**
     * Tell the motor what position it is actually at.
     * @param actualPosition The position the motor actually is at.
     */
    public void resetPosition(double actualPosition) {
        positionSetter.accept(actualPosition);
    }

    /**
     * @return The current position of the motor
     */
    public double getPosition() {
        return positionGetter.getAsDouble();
    }

    /**
     * @return The current velocity of the motor
     */
    public double getVelocity() {
        return velocityGetter.getAsDouble();
    }

    /**
     * Gets a set of sysID commands to run to characterize a mechanism
     * @param name Mechanism name
     * @param voltageRampRate The rate to increase volts at when running quasistatic tests, in volts/sec
     * @param stepVoltage The constant voltage to apply when running dynamic tests, in volts
     * @param timeout How long the tests should last, in seconds
     * @return a set of SysIDCommands
     */
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

    /**
     * Very similar to {@link #getSysIDCommands getSysIDCommands}
     * but for if you have multiple motors in one mechanism that are linked
     * <p>
     * Examples: Shooter with two motors driving 1 axle, 
     * or a drivetrain with 4 motors driving one robot
     * @param name Mechanism name
     * @param voltageRampRate The rate to increase volts at when running quasistatic tests, in volts/sec
     * @param stepVoltage The constant voltage to apply when running dynamic tests, in volts
     * @param timeout How long the tests should last, in seconds
     * @param otherMotors What other motors should also be synchronized for the test
     * @return The commands to run
     */
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

    /**
     * @return Whether we are about at the target specified by {@link #setTarget}
     * @implNote This method returns true when using voltage control with {@link #setVoltage}
     */
    public abstract boolean atTarget();

    @Override
    public abstract void periodic();
}
