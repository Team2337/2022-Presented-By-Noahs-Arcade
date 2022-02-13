package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREUtils;
import frc.robot.Constants;
import frc.robot.subsystems.TCS34275.RawColor;

/**
 * Subsystem for the delivery mechanism
 *
 * @author Michael F, Nicholas S, Alex C
 */
public class Delivery extends SubsystemBase {

  public static enum Direction {
    CLOCKWISE,
    COUNTER_CLOCKWISE
  }

  private final TalonFX motor = new TalonFX(Constants.DELIVERY_MOTOR_ID);

  private final TCSColorSensor sensor = new TCSColorSensor(I2C.Port.kMXP);

  private RawColor rawColor;
  private Color color;

  /**
   * Initializes the Delivery subsystem - no color sensors yet.
   */
  public Delivery() {
    motor.configFactoryDefault();

    motor.setNeutralMode(NeutralMode.Brake);
    motor.configOpenloopRamp(0.5);

    motor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit(), 0);

    // Set up shuffleboard stuff
    ShuffleboardTab deliveryTab = Shuffleboard.getTab("Delivery");
    ShuffleboardLayout infoWidget = deliveryTab.getLayout("Info", BuiltInLayouts.kList)
      .withSize(4, 8)
      .withPosition(0, 0);
    infoWidget.addNumber("Speed (%)", () -> motor.getMotorOutputPercent());
    infoWidget.addNumber("Temperature (C)", () -> motor.getTemperature());

    ShuffleboardLayout colorWidget = deliveryTab.getLayout("Sensor", BuiltInLayouts.kList)
      .withSize(4, 8)
      .withPosition(4, 0);
    colorWidget.addString("Color", () -> String.valueOf(sensor.getColor()));
    colorWidget.addNumber("Average", () -> {
      return (double)(rawColor.red+rawColor.green+rawColor.blue) / 3.0;
    });
    colorWidget.addNumber("Red", () -> rawColor.red);
    colorWidget.addNumber("Green", () -> rawColor.green);
    colorWidget.addNumber("Blue", () -> rawColor.blue);
    colorWidget.addNumber("Clear", () -> rawColor.clear);
    colorWidget.addNumber("Luminance", () -> rawColor.luminance);
    colorWidget.addNumber("Sum", () -> {return rawColor.red + rawColor.green + rawColor.blue;});
    colorWidget.addDoubleArray("Numbers", () -> new double[]{color.red, color.green, color.blue});
  }

  @Override
  public void periodic() {
    rawColor = sensor.getSensor().getRawColor();
    color = sensor.getSensor().getColor();
  }

  public void startDelivery(Direction direction) {
    startDelivery(direction, Constants.DELIVERY_SPEED);
  }

  public void startDelivery(Direction direction, double speed) {
    switch (direction) {
      case CLOCKWISE:
        // Rotate motor forward
        motor.set(ControlMode.PercentOutput, speed);
        break;
      case COUNTER_CLOCKWISE:
        // Rotate motor CCW
        motor.set(ControlMode.PercentOutput, -speed);
        break;
    }
  }

  /**
   * Stops the delivery mechanism
   */
  public void stopDelivery() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }

}
