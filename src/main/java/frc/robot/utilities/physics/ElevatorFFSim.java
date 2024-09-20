package frc.robot.utilities.physics;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class ElevatorFFSim extends FFSim{
    public ElevatorFFSim(double kG, double kS, double kV, double kA, State initialState) {
        super(
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

    public ElevatorFFSim(double kG, double kS, double kV, double kA) {
        this(kG, kS, kV, kA, new State());
    }
}
