package frc.robot.subsystems;

import java.lang.ref.WeakReference;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoDrivableCommand;

public class AutoDrive extends SubsystemBase {
  
  private WeakReference<AutoDrivableCommand> commandReference;

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
   * Registered an AutoDrivableCommand to be called when an output is needed for
   * the SwerveDriveCommand. Note: Only one AutoDrivableCommand can be registered
   * at a time. Commands calling registerAutoDrivableCommand should require
   * the AutoDrive subsystem in order to prevent other commands from
   * registering for callbacks.
   *
   * @param command
   */
  public void registerAutoDrivableCommand(AutoDrivableCommand command) {
    this.commandReference = new WeakReference<AutoDrivableCommand>(command);
  }

  public void unregisterAutoDrivableCommand() {
    this.commandReference = null;
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
    if (commandReference == null) {
      return null;
    }

    AutoDrivableCommand command = commandReference.get();
    if (command == null) {
      return null;
    }
    return command.calculate(forward, strafe, isFieldOriented);
  }

  @Override
  public void periodic() {
    if (commandReference != null) {
      AutoDrivableCommand command = commandReference.get();
      if (command != null) {
        SmartDashboard.putString("AutoDrivable Command", command.toString());
      } else {
        SmartDashboard.putString("AutoDrivable Command", "N/A");
      }
    } else {
      SmartDashboard.putString("AutoDrivable Command", "N/A");
    }
  }

}
