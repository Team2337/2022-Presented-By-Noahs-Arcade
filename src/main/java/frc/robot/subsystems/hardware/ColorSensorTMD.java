package frc.robot.subsystems.hardware;

import com.playingwithfusion.TMD37003;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.BallColor;

/**
 * Wrapper for the TMD-37003 (included in SEN-36004 sensor)
 */
public class ColorSensorTMD {

  private final double COLOR_SENSOR_PROXIMITY = 0.1; //TODO: tune me
  
  // Sensor
  private final TMD37003 sensor;

  // Color matches
  private final ColorMatch colorMatcher = new ColorMatch();
  private static final Color kMatchRed  = new Color(0.275, 0.45, 0.275);
  private static final Color kMatchBlue = new Color(0.25, 0.45, 0.3);

  public ColorSensorTMD(Port i2cPort) {
    sensor = new TMD37003(i2cPort);
    
    // Color match
    colorMatcher.addColorMatch(kMatchRed);
    colorMatcher.addColorMatch(kMatchBlue);
  }

  public BallColor getColor() {
    // Get color and what its closest color is
    Color detectedColor = new Color(sensor.getRed(), sensor.getGreen(), sensor.getBlue());
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    // Check match
    if (!seesBall()) {
      // If not close enough, there is no ball
      return null;
    } else if (match.color == kMatchRed) {
      // Red ball
      return BallColor.RED;
    } else if (match.color == kMatchBlue) {
      // Blue ball
      return BallColor.BLUE;
    } else {
      return null;
    }
  }

  public Color getRawColor() {
    return new Color(sensor.getRed(), sensor.getGreen(), sensor.getBlue());
  }

  public double getProximity() {
    return sensor.getProximity();
  }

  public boolean seesBall() {
    return sensor.getProximity() < COLOR_SENSOR_PROXIMITY;
  }

}
