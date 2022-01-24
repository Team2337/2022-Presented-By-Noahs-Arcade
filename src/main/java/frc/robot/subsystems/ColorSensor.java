package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Colors;

/**
 * Subsystem for the REV Robotics Color Sensor V3. Plugs in to the I2C port.
 * 
 * @author Michael Francis
 * 
 * @apiNote https://github.com/REVrobotics/Color-Sensor-v3-Examples/tree/master/Java
 */
public class ColorSensor extends SubsystemBase {

  // The sensor
  private final ColorSensorV3 sensor;

  // Color matches
  private final ColorMatch colorMatcher;
  private final Color matchRed  = new Color(0.4529, 0.2117, 0.0980);
  private final Color matchBlue = new Color(0.1078, 0.2608, 0.3921);
  // private final Color matchYellow = new Color(0.4803, 0.4019, 0.0431);

  // Other variables
  private Colors currentColor;

  /**
   * Initializes a REV Robotics Color Sensor v3
   * @param port The I2C port the sensor is plugged into
   */
  public ColorSensor(I2C.Port port){
    // Set up sensor
    sensor = new ColorSensorV3(port);

    // Color match
    colorMatcher = new ColorMatch();
    colorMatcher.addColorMatch(matchRed);
    colorMatcher.addColorMatch(matchBlue);

    currentColor = Colors.None;

    // Shuffleboard stuff
    ShuffleboardTab tab = Shuffleboard.getTab("Color Sensor");

    ShuffleboardLayout matchWidget = tab.getLayout("Match", BuiltInLayouts.kList)
      .withPosition(4, 0)
      .withSize(4, 8);
    matchWidget.addString("Match", () -> {
      switch(getColor()){
        case Red:    return "Red";
        case Blue:   return "Blue";
        default:     return "None";
      }
    });
  }

  @Override
  public void periodic(){
    // Get color and what its closest color is
    Color detectedColor = sensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    // Check match
    if(sensor.getProximity() < Constants.COLOR_SENSOR_PROXIMITY) {
      // If not close enough, there is no ball
      currentColor = Colors.None;
    } else if (match.color == matchRed) {
      // Red ball
      currentColor = Colors.Red;
    } else if (match.color == matchBlue) {
      // Blue ball
      currentColor = Colors.Blue;
    } else {
      // No ball
      currentColor = Colors.None;
    }
  }

  public Colors getColor(){
    return currentColor;
  }

}
