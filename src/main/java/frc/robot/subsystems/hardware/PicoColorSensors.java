package frc.robot.subsystems.hardware;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.BallColor;
import frc.robot.subsystems.hardware.PicoCommunicator.RawColor;

/**
 * Abstraction for {@link PicoCommunicator} for easier and clearer use.
 *
 * @author Michael F
 */
public class PicoColorSensors {

  // Proximity where ball is roughly 2 inches away from sensor
  private final int PROXIMITY_THRESHOLD = 300; // TODO: tune me

  // Class to abstract
  private final PicoCommunicator pico = new PicoCommunicator();

  // Color matches
  private final ColorMatch colorMatcher = new ColorMatch();
  private static final Color kMatchRed  = new Color(0.275, 0.45, 0.275);
  private static final Color kMatchBlue = new Color(0.25, 0.45, 0.3);

  public PicoColorSensors() {
    // Set up color matching
    colorMatcher.addColorMatch(kMatchRed);
    colorMatcher.addColorMatch(kMatchBlue);
  }

  /**
   * Converts a {@link RawColor} to a color
   * @param rawColor The {@link RawColor} to convert
   * @return A color with values between 0 and 1 that all together add up to 1
   */
  private Color rawColorToColor(RawColor rawColor) {
    double r = (double) rawColor.red;
    double g = (double) rawColor.green;
    double b = (double) rawColor.blue;
    double sum = r + g + b;
    return new Color(r / sum, g / sum, b / sum);
  }

  /**
   * Matches a Color to a BallColor
   * @param color The {@link Color} object
   * @return A BallColor representing the closest allowed value the color is (red or blue)
   */
  private BallColor matchColor(Color color) {
    // Match color
    ColorMatchResult match = colorMatcher.matchClosestColor(color);

    if (match.color == kMatchRed) {
      // Red ball
      return BallColor.RED;
    } else if (match.color == kMatchBlue) {
      // Blue ball
      return BallColor.BLUE;
    }
    // We shouldn't be here, but just in case, return null.
    return null;
  }

  /**
   * @return The BallColor of what the left sensor sees or null if it doesn't see anything
   */
  public BallColor getLeftSensorBallColor() {
    if (!leftSensorSeesBall()) {
      return null;
    }
    return matchColor(rawColorToColor(pico.getRawColor0()));
  }

  /**
   * @return The BallColor of what the left sensor sees or null if it doesn't see anything
   */
  public BallColor getRightSensorBallColor() {
    if (!rightSensorSeesBall()) {
      return null;
    }
    return matchColor(rawColorToColor(pico.getRawColor1()));
  }

  /**
   * @return If the proximity of the left color sensor is within a threshold
   */
  public boolean leftSensorSeesBall() {
    return pico.getProximity0() > PROXIMITY_THRESHOLD;
  }

  /**
   * @return If the proximity of the right color sensor is within a threshold
   */
  public boolean rightSensorSeesBall() {
    return pico.getProximity1() > PROXIMITY_THRESHOLD;
  }

  /**
   * @return Whether the left sensor is connected or not
   */
  public boolean leftSensorIsConnected() {
    return pico.isSensor0Connected();
  }

  /**
   * @return Whether the left sensor is connected or not
   */
  public boolean rightSensorIsConnected() {
    return pico.isSensor1Connected();
  }

}
