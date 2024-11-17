package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.Loggable;

public class Superstructure implements Loggable {
    // Create objects for all non-drivebase subsystems
    private Mechanism2d robotMech;
    public Superstructure() {
        robotMech = new Mechanism2d(1.5, 1.5);
        configureMech();
    }

    private void configureMech() {
        // Append subsystem mechs
        SmartDashboard.putData("Robot Mech", robotMech);
    }

    public void log(String name) {
        // Call log() methods for contained subsystems
    }

    // Put Command Factories Here
}
