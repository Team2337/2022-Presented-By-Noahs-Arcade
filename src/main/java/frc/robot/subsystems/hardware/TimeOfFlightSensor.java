package frc.robot.subsystems.hardware;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.Status;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimeOfFlightSensor extends SubsystemBase {

  private final TimeOfFlight sensor = new TimeOfFlight(0);

  private double distanceInches;

  @Override
  public void periodic() {
  if (getStatus() == Status.Valid || getStatus() == Status.Invalid){
    if (getStatus() == Status.Valid) {
      // mm -> m, then m -> inches
      distanceInches = Units.metersToInches((getDistanceMM() / 1000));
    }
  }
  else {
    distanceInches = -1; //-1 can be our error code for if our sensor gets messed up
  }


    SmartDashboard.putNumber("TimeOfFlight/Distance (in)", getDistanceInches());
    SmartDashboard.putString("TimeOfFlight/Status", getStatus().toString());
  }

  private Status getStatus() {
    return sensor.getStatus();
  }

  private double getDistanceMM() {
    return sensor.getRange();
  }

  /**
   * Get the last read distance from the sensor in inches.
   * Note: If the sensor's status is no longer valid, we will
   * remember it's last returned distance.
   */
  public double getDistanceInches() {
    return distanceInches;
  }

}
