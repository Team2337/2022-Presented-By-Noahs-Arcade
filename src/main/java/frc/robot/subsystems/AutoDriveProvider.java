package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * AutoDriveProvider is a fake subsystem (no hardware) that allows for
 * autonomous control of the Drivetrain subsystem through the SwerveDriveCommand
 * without to require the Drivetrain. Used in autonomous by Commands to provide
 * forward/strafe values to the Drivetrain while still allowing us to use the
 * Heading subsystem in the SwerveDriveCommand to control the heading of our
 * robot.
 */
public class AutoDriveProvider extends SubsystemBase {

  public class AutoDrive {
    public double forward;
    public double strafe;
    public boolean isFieldOriented;

    public AutoDrive(double forward, double strafe, boolean isFieldOriented) {
      this.forward = forward;
      this.strafe = strafe;
      this.isFieldOriented = isFieldOriented;
    }
  }

  private AutoDrive autoDrive;

  /**
   * Set a robot-oriented forward and strafe value for auto-drive.
   * 
   * @param forward Forward value relative to the maximum speed of our drivetrain
   *                - between [-1.0, 1.0]. Postive values are forward.
   * @param strafe  Strafe value relative to the maximum speed of our drivetrain
   *                - between [-1.0, 1.0]. Positive values are left.
   */
  public void setAutoDrive(double forward, double strafe) {
    setAutoDrive(forward, strafe, false);
  }

  /**
   * Set a forward and strafe value for auto-drive.
   * 
   * @param forward         Forward value relative to the maximum speed of our
   *                        drivetrain
   *                        - between [-1.0, 1.0]. Postive values are forward.
   * @param strafe          Strafe value relative to the maximum speed of our
   *                        drivetrain
   *                        - between [-1.0, 1.0]. Positive values are left.
   * @param isFieldOriented Boolean indicating if the forward and strafe values
   *                        should generate field-oriented or robot-oriented
   *                        speeds. Field-oriented positive forward is towards the
   *                        opponents driver station, positive strafe values are
   *                        towards the left of the field.
   */
  public void setAutoDrive(double forward, double strafe, boolean isFieldOriented) {
    this.autoDrive = new AutoDrive(forward, strafe, isFieldOriented);
  }

  public void clearAutoDrive() {
    this.autoDrive = null;
  }

  /**
   * Get the latest auto-drive movements. Clears the most recent auto-drive values.
   *
   * @return An AutoDrive value or null if there is no current AutoDrive value.
   */
  public AutoDrive getAutoDrive() {
    AutoDrive autoDrive = this.autoDrive;
    // Clearing the previous auto-drive is important because it means we only queue up one
    // robot move at a time. If we want the robot to keep moving in the given direction,
    // a Command should continue feeding the AutoDriveProvider.
    clearAutoDrive();
    return autoDrive;
  }

}
