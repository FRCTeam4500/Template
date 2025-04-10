package frc.robot.programs.swerve;

import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.WiringConstants.SwerveWiring;
import frc.robot.programs.LoggedRobot;
import frc.robot.utilities.logging.HoundLog;

public class EncoderTest extends LoggedRobot {
  private AnalogEncoder fl;
  private AnalogEncoder fr;
  private AnalogEncoder bl;
  private AnalogEncoder br;

  public EncoderTest() {
    fl = new AnalogEncoder(SwerveWiring.FRONT_LEFT_ENCODER_ID);
    fr = new AnalogEncoder(SwerveWiring.FRONT_RIGHT_ENCODER_ID);
    bl = new AnalogEncoder(SwerveWiring.BACK_LEFT_ENCODER_ID);
    br = new AnalogEncoder(SwerveWiring.BACK_RIGHT_ENCODER_ID);
  }

  @Override
  public void robotPeriodic() {
    HoundLog.log("fl angle encoder", fl.get());
    HoundLog.log("fr angle encoder", fr.get());
    HoundLog.log("bl angle encoder", bl.get());
    HoundLog.log("br angle encoder", br.get());
  }
}
