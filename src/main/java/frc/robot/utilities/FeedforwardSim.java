package frc.robot.utilities;

import java.util.function.BiConsumer;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Motor.FeedforwardConstants;
import frc.robot.hardware.Motor.TargetType;

public class FeedforwardSim extends SubsystemBase {
    /** function that modifies the State argument, stepping forward in time by 20 ms */
    private BiConsumer<State, Double> calc;
    /** position and velocity */
    private State state;
    /** current voltage */
    private double volts;

    /**
     * @param calc A function that steps the state forward in time (by 20 ms)
     * @param initalState The inital state of the mechanism
     * @apiNote calc should modify its arguments
     */
    public FeedforwardSim(BiConsumer<State, Double> calc, State initalState) {
        this.calc = calc;
        this.state = initalState;
    }

    /**
     * If this Simulator is enabled, steps time forward by 20 ms.
     */
    public void periodic() {
        if (DriverStation.isDisabled()) {
            volts = 0;
        }
        calc.accept(state, volts);
    }


    /** setter function
     * @param volts new voltage
     */
    public void setVoltage(double volts) {
        this.volts = volts;
    }

    /** voltage getter */
    public double getVoltage() {
        return volts;
    }

    /** position getter */
    public double getPosition() {
        return state.position;
    }

    /** velocity getter */
    public double getVelocity() {
        return state.velocity;
    }

    /** changes position */
    public void resetPosition(double newPosition) {
        state.position = newPosition;
    }

    /**
     * Creates a feedforward sim model for a flywheel mechanism. 
     * This is also applicable for systems that extend instead of rotating,
     * as long as they aren't affected by gravity. For example, a horizontal elevator.
     * <p>
     * The feedforward constants should be obtained via SysId
     * @param kS The voltage needed to overcome the friction forces in the system.
     * @param kV The voltage needed to cause a given constant velocity.
     * @param kA The voltage needed to cause a given acceleration
     * @param initialState The inital position and velocity of the mechanism
     * @implNote this is just {@link #createElevator createElevator} but where gravity is 0.
     */
    public static FeedforwardSim createFlywheel(double kS, double kV, double kA, State initialState) {
        return createElevator(0, kS, kV, kA, initialState);
    }

    /**
     * Creates a feedforward sim model for a elevator mechanism. 
     * This is applicable for systems that extends against a constant force of gravity.
     * <p>
     * The feedforward constants should be obtained via SysId
     * @param kG The voltage need to overcome the gravitational force on the system.
     * @param kS The voltage needed to overcome the friction forces in the system.
     * @param kV The voltage needed to cause a given constant velocity.
     * @param kA The voltage needed to cause a given acceleration.
     * @param initialState The inital position and velocity of the mechanism.
     */
    public static FeedforwardSim createElevator(double kG, double kS, double kV, double kA, State initialState) {
        return new FeedforwardSim(
            (state, volts) -> {
                double staticVolts = Math.signum(state.velocity) * kS;
                double velocityVolts = state.velocity * kV;
                double deltaVel = 0.02 * (volts - kG - staticVolts - velocityVolts) / kA;
                double averageVel = state.velocity + deltaVel / 2;
                state.position += 0.02 * averageVel;
                state.velocity += deltaVel;
            }, initialState
        );
    }

    /**
     * Creates a feedforward sim model for a jointed arm mechanism. 
     * This is applicable for systems that rotate vertically, and face different
     * gravitational forces depending on their angle. Note that 0 rotations
     * must corespond to the arm being parallel to the ground
     * <p>
     * The feedforward constants should be obtained via SysId
     * @param kG The voltage need to overcome the gravitational force on the system
     * when the mechanism is parallel to the ground (0 rotations).
     * @param kS The voltage needed to overcome the friction forces in the system.
     * @param kV The voltage needed to cause a given constant velocity.
     * @param kA The voltage needed to cause a given acceleration.
     * @param initialState The inital position and velocity of the mechanism in rotations and rotations/second.
     */
    public static FeedforwardSim createArm(double kG, double kS, double kV, double kA, State initialState) {
        return new FeedforwardSim(
            (state, volts) -> {
                double gravityVolts = Math.cos(state.position * 2 * Math.PI) * kG;
                double staticVolts = Math.signum(state.velocity) * kS;
                double velocityVolts = state.velocity * kV;
                double deltaVel = 0.02 * (volts - gravityVolts - staticVolts - velocityVolts) / kA;
                double averageVel = state.velocity + deltaVel / 2;
                state.position += 0.02 * averageVel;
                state.velocity += deltaVel;
            }, initialState  
        );
    }

    /**
     * Creates a feedforward sim for the given type. If the type is rotation, 
     * {@link #createArm} is returned. Otherwise, {@link #createElevator} is returned
     * @param type What the goal of the mechansim is.
     * @param ff The feedforward values used to simulate 
     * @param initalState The inital positon and velocity of the mechanism
     * @return
     */
    public static FeedforwardSim create(TargetType type, FeedforwardConstants ff, State initalState) {
        if (type == TargetType.Rotation) {
            return createArm(ff.kG(), ff.kS(), ff.kV(), ff.kA(), initalState);
        } else {
            return createElevator(ff.kG(), ff.kS(), ff.kV(), ff.kA(), initalState);
        }
    }
}
