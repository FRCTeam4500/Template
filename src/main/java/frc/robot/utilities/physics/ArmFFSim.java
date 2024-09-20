package frc.robot.utilities.physics;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class ArmFFSim extends FFSim {
    public ArmFFSim(double kG, double kS, double kV, double kA, State initialState) {
        super(
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

    public ArmFFSim(double kG, double kS, double kV, double kA) {
        this(kG, kS, kV, kA, new State());
    }
}
