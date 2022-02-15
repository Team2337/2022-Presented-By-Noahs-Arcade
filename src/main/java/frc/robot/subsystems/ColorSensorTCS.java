package frc.robot.subsystems;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallColor;
import frc.robot.subsystems.TCSSensor.RawColor;

/**
 * Subsystem for the TCS34275 color sensor. Plugs in to the I2C port. Based on {@link ColorSensorREV}
 * 
 * @author Michael Francis
 */
public class ColorSensorTCS extends SubsystemBase {

  // private final int COLOR_SENSOR_PROXIMITY = 10; //TODO: find value of this
  private final double LUMINANCE_MAX_THRESHOLD = 96.0;

  // The sensor
  private final TCSSensor sensor;

  // Color matches
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color matchRed  = new Color(0.40, 0.30, 0.30);
  private final Color matchBlue = new Color(0.20, 0.40, 0.40);
  private final Color matchBlue2 = new Color(0.25, 0.35, 0.40);
  private final Color matchBlack = new Color(0.3333, 0.3333, 0.3333);

  // Other variables
  private BallColor currentColor = null;

  /**
   * Initializes a REV Robotics Color Sensor v3
   * @param port The I2C port the sensor is plugged into
   */
  public ColorSensorTCS(I2C.Port port) {
    // Set up sensor with a specific port
    sensor = new TCSSensor(port);

    // Color match
    colorMatcher.addColorMatch(matchRed);
    colorMatcher.addColorMatch(matchBlue);
    colorMatcher.addColorMatch(matchBlue2);
    colorMatcher.addColorMatch(matchBlack);
  }

  @Override
  public void periodic() {
    // Get color and what its closest color is
    Color detectedColor = sensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    // Reset current color
    currentColor = null;

    // Check match
    if (!seesBall()) {
      // If not close enough, there is no ball
      return;
    } else if (match.color == matchRed) {
      // Red ball
      currentColor = BallColor.RED;
    } else if (match.color == matchBlue || match.color == matchBlue2) {
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
    // TODO: this sensor doesn't return proximity, figure out how to do this

    // Luminance max threshold
    return sensor.getRawColor().luminance < LUMINANCE_MAX_THRESHOLD;
  }

  public TCSSensor getSensor() {
    return sensor;
  }

}
