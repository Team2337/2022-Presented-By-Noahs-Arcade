package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for the REV Robotics Color Sensor V3. Plugs in to the I2C port.
 * 
 * @author Michael Francis
 * 
 * @apiNote https://github.com/REVrobotics/Color-Sensor-v3-Examples/tree/master/Java
 */
public class ColorSensor extends SubsystemBase {
  
  /** The I2C port on the roboRIO */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 sensor;

  private Color detectedColor;
  private double infrared;
  private int proximity;

  public ColorSensor(){
    sensor = new ColorSensorV3(i2cPort);

    // Shuffleboard stuff
    ShuffleboardTab tab = Shuffleboard.getTab("Color Sensor");

    ShuffleboardLayout widget = tab.getLayout("Info", BuiltInLayouts.kList).withPosition(4, 0).withSize(3, 6);
    widget.addNumber("Red", () -> detectedColor.red);
    widget.addNumber("Green", () -> detectedColor.green);
    widget.addNumber("Blue", () -> detectedColor.blue);
    widget.addNumber("Infrared", () -> infrared);
    widget.addNumber("Proximity", () -> proximity);
  }

  @Override
  public void periodic(){
    // Update values
    detectedColor = sensor.getColor();
    infrared = sensor.getIR();
    proximity = sensor.getProximity();
  }


  /**
   * @return A Color object of the detected color value
   */
  public Color getColor(){
    return detectedColor;
  }

  /**
   * @return The raw color value from the IR ADC
   */
  public double getIR(){
    return infrared;
  }

  /**
   * @return The proximity of the detected object from 0-2047 where larger is closer
   */
  public int getProximity(){
    return proximity;
  }

}
