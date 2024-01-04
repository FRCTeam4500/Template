package frc.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.GamePieceVision;

public class WatchForGamepieceCommand extends CommandBase {

    GamePieceVision vision;

    public WatchForGamepieceCommand() {
        vision = GamePieceVision.getInstance();
    }

    @Override
    public void execute() {
        
    }
    
}
