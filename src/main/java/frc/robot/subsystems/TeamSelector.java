package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TeamSelector extends SubsystemBase {
  static DigitalInput teamSwitch = new DigitalInput(Constants.TEAMSWITCH_ID);
  static String sendSwitchData;

  public TeamSelector() {}

  @Override
  public void periodic() {
    TeamSelector.getTeamColor();
  }

  public static boolean getTeamColor() {
    sendSwitchData = !teamSwitch.get() ? "Red" : "Blue";
    SmartDashboard.putString("Which Team", sendSwitchData);
    return !teamSwitch.get(); // true if red, false if blue
  }
}
