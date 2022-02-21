package frc.robot.subsystems.hardware;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.Status;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimeOfFlightSensor extends SubsystemBase {

  private final TimeOfFlight sensor = new TimeOfFlight(0);
  
  private double distanceInches;
  private int validCounter = 0;

  private final int MAX_VALID_COUNTER = 3;

/*Hardware Failure
Hardware failure
InternalError	
Internal algorithm underflow or overflow
Invalid	
The measured distance is invalid
ReturnPhaseBad	
Return signal phase is out of bounds.
ReturnSignalLow	
Return signal value is below the internal defined threshold.
SigmaHigh	
Sigma estimator check is above internally defined threshold.
Valid	
Measured distance is valid
WrappedTarget	
Wrapped target, non-matching phases.*/

  @Override
  public void periodic() {
    if (getStatus() == Status.Valid) {
      validCounter = 0;
      // mm -> m, then m -> inches
      distanceInches = Units.metersToInches((getDistanceMM() / 1000));
    } else if (validCounter >= MAX_VALID_COUNTER) {
      distanceInches = -1;
    }

    validCounter++;

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
