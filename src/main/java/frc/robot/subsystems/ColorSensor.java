package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallColor;

/**
 * Subsystem for the REV Robotics Color Sensor V3. Plugs in to the I2C port.
 * 
 * @author Michael Francis
 * 
 * @apiNote https://github.com/REVrobotics/Color-Sensor-v3-Examples/tree/master/Java
 */
public class ColorSensor extends SubsystemBase {

  private final int COLOR_SENSOR_PROXIMITY = 300; //TODO: test number in delivery, see if there are actual units here

  // The sensor
  private final ColorSensorV3 sensor;

  // Color matches
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color matchRed  = new Color(0.4529, 0.2117, 0.0980);
  private final Color matchBlue = new Color(0.1078, 0.2608, 0.3921);

  // Other variables
  private BallColor currentColor = null;

  /**
   * Initializes a REV Robotics Color Sensor v3
   * @param port The I2C port the sensor is plugged into
   */
  public ColorSensor(I2C.Port port) {
    // Set up sensor with a specific port
    sensor = new ColorSensorV3(port);

    // Color match
    colorMatcher.addColorMatch(matchRed);
    colorMatcher.addColorMatch(matchBlue);
  }

  @Override
  public void periodic() {
    // Get color and what its closest color is
    Color detectedColor = sensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    // Reset current color
    currentColor = null;

    // Check match
    if (seesBall()) {
      // If not close enough, there is no ball
      return;
    } else if (match.color == matchRed) {
      // Red ball
      currentColor = BallColor.RED;
    } else if (match.color == matchBlue) {
      // Blue ball
      currentColor = BallColor.BLUE;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ColorSensor");
    builder.addStringProperty("Match", () -> {
      switch(getColor()){
        case RED:    return "Red";
        case BLUE:   return "Blue";
        default:     return "None";
      }
    }, null);
  }

  /**
   * @return Currently viewed color, if any
   */
  public BallColor getColor() {
    return currentColor;
  }

  /**
   * @return Whether or not the proximity sensor detects a close object
   */
  public boolean seesBall() {
    return sensor.getProximity() < COLOR_SENSOR_PROXIMITY;
  }

}
