package frc.robot.utilities;
public class CustomMotionMagic {
    private double m_maxVelocity;
    private double m_acceleration;
    private double m_targetPosition;
    private double m_positionThreshold;
    private double m_velocityThreshold;
    private double m_loopTime;
    private boolean m_doContinuousRotation;
    private double m_angleUnitsPerRevolution;
    public CustomMotionMagic(double maxVelocityConstraint, double accelerationConstraint) {
        m_maxVelocity = Math.abs(maxVelocityConstraint);
        m_acceleration = Math.abs(accelerationConstraint);
        m_loopTime = 0.02;
        m_positionThreshold = m_velocityThreshold = m_acceleration * m_loopTime;
    }

    public void setTargetPosition(double newTargetPosition) {
        m_targetPosition = newTargetPosition;
    }

    public void setPositionThreshold(double newPositionThreshold) {
        m_positionThreshold = Math.abs(newPositionThreshold);
    }

    public void setVelocityThreshold(double newVelocityThreshold) {
        m_velocityThreshold = Math.abs(newVelocityThreshold);
    }

    private double changeVelocity(double currentVelocity, double direction) {
        return HelperMethods.clampAroundZero(currentVelocity + Math.signum(direction) * m_acceleration * m_loopTime, m_maxVelocity);
    }

    public void enableContinuousRotation(double angleUnitsPerRevolution) {
        m_doContinuousRotation = true;
        m_angleUnitsPerRevolution = angleUnitsPerRevolution;
    }

    public void disableContinuousRotation() {
        m_doContinuousRotation = false;
    }

    public double calcDistToTarget(double currentPosition) {
        if (!m_doContinuousRotation) {
            return m_targetPosition - currentPosition;
        }
        double wrappedCurrentRotation = currentPosition % m_angleUnitsPerRevolution;
        double wrappedTargetRotation = m_targetPosition % m_angleUnitsPerRevolution;
        if (wrappedCurrentRotation < 0) {
            wrappedCurrentRotation += m_angleUnitsPerRevolution;
        }
        if (wrappedTargetRotation < 0) {
            wrappedTargetRotation += m_angleUnitsPerRevolution;
        }
        double differenceInRotations = wrappedTargetRotation - wrappedCurrentRotation; 
        if (Math.abs(differenceInRotations) < Math.abs(m_angleUnitsPerRevolution + differenceInRotations)) {
            return differenceInRotations;
        }
        return m_angleUnitsPerRevolution + differenceInRotations;
    }

    public double calcNextVelocity(double currentPosition, double currentVelocity) {
        if (atTarget(currentPosition, currentVelocity)) {
            return 0;
        }
        double distanceToTarget = calcDistToTarget(currentPosition);
        double positionChangeWhileSlowDown = Math.pow(changeVelocity(currentVelocity, distanceToTarget), 2) / (2 * m_acceleration);
        if (Math.abs(distanceToTarget) <= positionChangeWhileSlowDown) {
            return changeVelocity(currentVelocity, -currentVelocity);
        }
        return changeVelocity(currentVelocity, distanceToTarget);
    }

    public boolean atTarget(double currentPosition, double currentVelocity) {
        return Math.abs(calcDistToTarget(currentPosition)) <= m_positionThreshold && Math.abs(currentVelocity) <= m_velocityThreshold;
    }
}
