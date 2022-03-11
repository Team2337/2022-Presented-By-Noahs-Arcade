package frc.robot.subsystems.hardware;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.util.Units;

public class TimeOfFlightSensor {

  private final TimeOfFlight sensor = new TimeOfFlight(0);

  public TimeOfFlightSensor() {
    sensor.setRangingMode(RangingMode.Short, 24);
    sensor.setRangeOfInterest(6, 6, 10, 10);
  }

  /**
   * Get the last read distance from the sensor in inches.
   */
  public double getDistanceInches() {
    return Units.metersToInches((getDistanceMM() / 1000));
  }

  private double getDistanceMM() {
    return sensor.getRange();
  }

  /**
   * @return True if everything is alright
   */
  public boolean systemsCheck() {
    return getDistanceInches() > 0.0;
  }

}
