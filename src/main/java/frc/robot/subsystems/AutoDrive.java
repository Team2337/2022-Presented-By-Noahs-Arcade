package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoDrivableCommand;

public class AutoDrive extends SubsystemBase {
  
  public static class State {
    public double forward;
    public double strafe;
    public boolean isFieldOriented;

    public State(double forward, double strafe, boolean isFieldOriented) {
      this.forward = forward;
      this.strafe = strafe;
      this.isFieldOriented = isFieldOriented;
    }
  }

  /**
   * Calculate forward/strafe values using the current command. May return null if
   * there is no auto drive command scheduled or the currently auto drive command
   * does not specify an auto drive state.
   *
   * @param forward         - The user-initiated forward
   * @param strafe          - The user-initiated strafe
   * @param isFieldOriented - The user-initiated isFieldOriented
   * @return - A negotiated forward/strafe from the auto driveable command
   */
  public State calculate(double forward, double strafe, boolean isFieldOriented) {
    AutoDrivableCommand command = getAutoDrivableCommand();
    if (command == null) {
      return null;
    }
    return command.calculate(forward, strafe, isFieldOriented);
  }

  /**
   * Gets the current Command for the subsystem if it's an AutoDrivableCommand.
   * May return null if the subsystem has no Command or the Command does
   * not conform to AutoDrivableCommand.
   */
  private AutoDrivableCommand getAutoDrivableCommand() {
    Command command = getCurrentCommand();
    if (command != null) {
      SmartDashboard.putData("AutoDrive Current Command", (CommandBase) command);
      SmartDashboard.putBoolean("Is AutoDrivableCommand", command instanceof AutoDrivableCommand);
    }
    if (command != null && command instanceof AutoDrivableCommand) {
      return (AutoDrivableCommand) command;
    }
    return null;
  }

}
