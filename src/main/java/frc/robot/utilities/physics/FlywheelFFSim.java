package frc.robot.utilities.physics;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class FlywheelFFSim extends FFSim {
    public FlywheelFFSim(
        double kS,
        double kV,
        double kA,
        State initalState
    ) {
        super(
            (state, voltage) -> {
                double deltaVel = 0.02 * (voltage - state.velocity * kV - Math.signum(voltage) * kS) / kA;
                double averageVel = state.velocity + deltaVel / 2;
                state.position += 0.02 * averageVel;
                state.velocity += deltaVel;
                
            }, initalState
        );
    }

    public FlywheelFFSim(double kS, double kV, double kA) {
        this(kS, kV, kA, new State());
    }
}
