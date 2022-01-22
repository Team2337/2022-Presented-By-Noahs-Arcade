package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for the REV Robotics Color Sensor V3. Plugs in to the I2C port.
 * <p>
 * TODO: Color matching
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

  private ShuffleboardComponent<SuppliedValueWidget<Boolean>> colorWidget;

  public ColorSensor(){
    sensor = new ColorSensorV3(i2cPort);

    // Shuffleboard stuff
    ShuffleboardTab tab = Shuffleboard.getTab("Color Sensor");

    // General info
    ShuffleboardLayout infoWidget = tab.getLayout("Info", BuiltInLayouts.kList).withPosition(4, 0).withSize(4, 8);
    infoWidget.addNumber("Red", () -> detectedColor.red);
    infoWidget.addNumber("Green", () -> detectedColor.green);
    infoWidget.addNumber("Blue", () -> detectedColor.blue);
    infoWidget.addNumber("Infrared", () -> infrared);
    infoWidget.addNumber("Proximity", () -> proximity);

    // Show color
    colorWidget = tab.addBoolean("Color", () -> true).withPosition(8, 0).withSize(4, 4);
  }

  @Override
  public void periodic(){
    // Update values
    detectedColor = sensor.getColor();
    infrared = sensor.getIR();
    proximity = sensor.getProximity();

    // Update color on Shuffleboard
    // https://www.chiefdelphi.com/t/shuffleboard-color-block-for-color-sensor/372871/4
    // colorWidget.withProperties(Map.of("colorWhenTrue", detectedColor));
    colorWidget.withProperties(
      Map.of(
        "Color when true",
        String.format(
          "#%02x%02x%02x",
          (int)Math.floor(detectedColor.red * 255),
          (int)Math.floor(detectedColor.green * 255),
          (int)Math.floor(detectedColor.blue * 255)
        )
      )
    );
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
