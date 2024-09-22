package frc.robot.utilities.physics;

import java.util.function.BiConsumer;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class FeedforwardSim {
    private BiConsumer<State, Double> calc;
    private State state;
    private double volts;

    /**
     * @param calc A function that takes the current state and the voltage, and
     * modifies the passed in state to be the state after 0.02 seconds
     * @param initalState The inital state of the mechanism
     */
    public FeedforwardSim(BiConsumer<State, Double> calc, State initalState) {
        this.calc = calc;
        this.state = initalState;
    }

    /**
     * Must be called every loop
     */
    public void periodic() {
        calc.accept(state, volts);
    }

    public void setVoltage(double volts) {
        this.volts = volts;
    }

    public double getVoltage() {
        return volts;
    }

    public double getPosition() {
        return state.position;
    }

    public double getVelocity() {
        return state.velocity;
    }

    public static FeedforwardSim createFlywheel(double kS, double kV, double kA, State initalState) {
        return new FeedforwardSim(
            (state, voltage) -> {
                double staticVolts = Math.signum(voltage) * kS;
                double velocityVolts = state.velocity * kV;
                double deltaVel = 0.02 * (voltage - staticVolts - velocityVolts) / kA;
                double averageVel = state.velocity + deltaVel / 2;
                state.position += 0.02 * averageVel;
                state.velocity += deltaVel;
                
            }, initalState
        );
    }

    public static FeedforwardSim createElevator(double kG, double kS, double kV, double kA, State initialState) {
        return new FeedforwardSim(
            (state, volts) -> {
                double staticVolts = Math.signum(volts) * kS;
                double velocityVolts = state.velocity * kV;
                double deltaVel = 0.02 * (volts - kG - staticVolts - velocityVolts) / kA;
                double averageVel = state.velocity + deltaVel / 2;
                state.position += 0.02 * averageVel;
                state.velocity += deltaVel;
            }, initialState
        );
    }

    public static FeedforwardSim createArm(double kG, double kS, double kV, double kA, State initialState) {
        return new FeedforwardSim(
            (state, volts) -> {
                double gravityVolts = Math.cos(state.position * 2 * Math.PI) * kG;
                double staticVolts = Math.signum(volts) * kS;
                double velocityVolts = state.velocity * kV;
                double deltaVel = 0.02 * (volts - gravityVolts - staticVolts - velocityVolts) / kA;
                double averageVel = state.velocity + deltaVel / 2;
                state.position += 0.02 * averageVel;
                state.velocity += deltaVel;
            }, initialState  
        );
    }
}
