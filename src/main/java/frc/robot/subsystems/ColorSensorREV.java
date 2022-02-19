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
public class ColorSensorREV extends SubsystemBase implements ColorSensor {

  private final int COLOR_SENSOR_PROXIMITY = 300; //TODO: tune me

  // The sensor
  private final ColorSensorV3 sensor;

  // Color matches
  private final ColorMatch colorMatcher = new ColorMatch();
  private static final Color kMatchRed  = new Color(0.4529, 0.2117, 0.0980);
  private static final Color kMatchBlue = new Color(0.1078, 0.2608, 0.3921);

  // Other variables
  private BallColor currentColor = null;

  /**
   * Initializes a REV Robotics Color Sensor v3
   * @param port The I2C port the sensor is plugged into
   */
  public ColorSensorREV(I2C.Port port) {
    // Set up sensor with a specific port
    sensor = new ColorSensorV3(port);

    // Color match
    colorMatcher.addColorMatch(kMatchRed);
    colorMatcher.addColorMatch(kMatchBlue);
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
    } else if (match.color == kMatchRed) {
      // Red ball
      currentColor = BallColor.RED;
    } else if (match.color == kMatchBlue) {
      // Blue ball
      currentColor = BallColor.BLUE;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ColorSensorREV");
    builder.addStringProperty("Match", () -> {
      switch(getColor()) {
        case RED:
          return "Red";
        case BLUE:
          return "Blue";
        default:
          return "None";
      }
    }, null);
  }

  public BallColor getColor() {
    return currentColor;
  }

  public boolean seesBall() {
    return sensor.getProximity() > COLOR_SENSOR_PROXIMITY;
  }

}
