package frc.robot.subsystems.hardware;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallColor;
import frc.robot.subsystems.hardware.interfaces.ColorSensor;

/**
 * Subsystem for the TCS34275 color sensor. Plugs in to the I2C port. Based on {@link ColorSensorREV}
 *
 * @author Michael Francis
 */
public class ColorSensorTCS extends SubsystemBase implements ColorSensor {

  private final double LUMINANCE_MAX_THRESHOLD = 12.0; //TODO: tune this

  // The sensor
  private final TCSSensor sensor;

  // Color matches
  private final ColorMatch colorMatcher = new ColorMatch();
  private static final Color kMatchRed  = new Color(0.40, 0.30, 0.30);
  private static final Color kMatchBlue = new Color(0.20, 0.35, 0.45);
  private static final Color kMatchBlack = new Color(0.3333, 0.3333, 0.3333);

  // Other variables
  private BallColor currentColor = null;

  /**
   * Initializes a TCS34725 color sensor
   * @param port The I2C port the sensor is plugged into
   */
  public ColorSensorTCS(I2C.Port port) {
    // Set up sensor with a specific port
    sensor = new TCSSensor(port);

    // Color match
    colorMatcher.addColorMatch(kMatchRed);
    colorMatcher.addColorMatch(kMatchBlue);
    colorMatcher.addColorMatch(kMatchBlack);
  }

  @Override
  public void periodic() {
    // Get color and what its closest color is
    long start = System.nanoTime();
    Color detectedColor = sensor.getColor();
    long end = System.nanoTime();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    System.out.println("Delivery time: " + String.valueOf((double)(end - start) / 1000000.0));

    // Reset current color
    currentColor = null;

    // Check match
    if (!seesBall()) {
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

  public BallColor getColor() {
    return currentColor;
  }

  public boolean seesBall() {
    /**
     * Luminance max threshold
     *
     * This is a terrible way to go about this. If we can find a better way to do this, such as
     * using a Time of Flight sensor instead (there is code in the 2020 repo), it would be much
     * better. The current solution works, but it's a very sketchy solution.
     */
    return sensor.getRawColor().luminance < LUMINANCE_MAX_THRESHOLD;
  }

}
