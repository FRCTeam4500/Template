package frc.robot.utilities.physics;

import java.util.function.BiConsumer;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class FFSim {
    private BiConsumer<State, Double> calc;
    private State state;
    private double volts;

    public FFSim(BiConsumer<State, Double> calc, State initalState) {
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
}
