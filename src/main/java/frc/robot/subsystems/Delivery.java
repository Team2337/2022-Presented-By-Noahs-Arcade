package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.SystemsCheckPositions;
import frc.robot.nerdyfiles.utilities.CTREUtils;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.hardware.PicoColorSensors;

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

  private final TalonFX motor = new TalonFX(
    Constants.DELIVERY_MOTOR_ID,
    Constants.UPPER_CANIVORE_ID
  );

  // Color sensors
  private final PicoColorSensors colorSensors = new PicoColorSensors();

  // Beam break sensor
  private final DigitalInput shooterBeam = new DigitalInput(Constants.SHOOTER_BEAM_ID);

  private final DigitalInput ballCenteringSensor = new DigitalInput(Constants.getInstance().CENTERING_BEAM_ID);

  private int balls = 0;

  /**
   * Initializes the Delivery subsystem.
   */
  public Delivery() {
    motor.configFactoryDefault();

    motor.setNeutralMode(NeutralMode.Brake);
    motor.configOpenloopRamp(0.5);

    motor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit(), 0);

    setupShuffleboard(Constants.DashboardLogging.DELIVERY);
  }

  private void setupShuffleboard(Boolean logEnable) {
    if (logEnable) {
      ShuffleboardTab deliveryTab = Shuffleboard.getTab("Delivery");

      ShuffleboardLayout infoWidget = deliveryTab.getLayout("Info", BuiltInLayouts.kList)
        .withSize(6, 8)
        .withPosition(0, 0);
      infoWidget.addNumber("Speed (%)", () -> motor.getMotorOutputPercent());
      infoWidget.addNumber("Temperature (F)", () -> Utilities.convertCelsiusToFahrenheit(motor.getTemperature()));
      infoWidget.addNumber("Balls", () -> balls);
      infoWidget.addBoolean("Sees Ball", this::getLeftColorSensorStatus);

      ShuffleboardLayout sensorsWidget = deliveryTab.getLayout("Sensors and States", BuiltInLayouts.kList)
        .withSize(6, 8)
        .withPosition(6, 0);
      sensorsWidget.addStringArray("Color sensors", () -> new String[]{
        "Left: " + String.valueOf(colorSensors.getLeftSensorBallColor()),
        "Right: " + String.valueOf(colorSensors.getRightSensorBallColor())
      });
      sensorsWidget.addStringArray("Proximities", () -> new String[]{
        "Left: " + String.valueOf(colorSensors.leftSensorSeesBall()),
        "Right: " + String.valueOf(colorSensors.rightSensorSeesBall())
      });
      sensorsWidget.addStringArray("Other sensors", () -> new String[]{
        "Centering Sensor: "   + getCenteringSensorStatus()
      });
    }

    // Systems check
    if (Constants.DO_SYSTEMS_CHECK) {
      ShuffleboardTab systemsCheck = Constants.SYSTEMS_CHECK_TAB;

      systemsCheck.addBoolean("Left Color Sensor", () -> colorSensors.leftSensorIsConnected())
        .withPosition(SystemsCheckPositions.L_COLOR_SENSOR.x, SystemsCheckPositions.L_COLOR_SENSOR.y)
        .withSize(3, 3);
      systemsCheck.addBoolean("Right Color Sensor", () -> colorSensors.rightSensorIsConnected())
        .withPosition(SystemsCheckPositions.R_COLOR_SENSOR.x, SystemsCheckPositions.R_COLOR_SENSOR.y)
        .withSize(3, 3);

      systemsCheck.addNumber("Delivery Temp (°C)", () -> getTemperature())
        .withPosition(SystemsCheckPositions.DELIVERY_TEMP.x, SystemsCheckPositions.DELIVERY_TEMP.y)
        .withSize(3, 4)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", Constants.MOTOR_MINIMUM_TEMP_CELSIUS, "Max", Constants.MOTOR_SHUTDOWN_TEMP_CELSIUS));

      systemsCheck.addBoolean("Centering Sensor", () -> getCenteringSensorStatus())
        .withPosition(SystemsCheckPositions.CENTERING_SENSOR.x, SystemsCheckPositions.CENTERING_SENSOR.y)
        .withSize(3, 3);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Centering Sensor Status", getCenteringSensorStatus());
  }


  ///////////////////////////
  // --------------------- //
  // --- MOTOR METHODS --- //
  // --------------------- //
  ///////////////////////////

  public void start(Direction direction) {
    setSpeed(direction, Constants.DELIVERY_SPEED);
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

  public void setSpeed(Direction direction, double speed) {
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

  public double getTemperature() {
    return motor.getTemperature();
  }

  /**
   * Stops the delivery mechanism
   */
  public void stop() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }


  ////////////////////////////////////
  // ------------------------------ //
  // --- ORGANIZATIONAL METHODS --- //
  // ------------------------------ //
  ////////////////////////////////////

  /**
   * Adds a new ball to the bottom of the delivery internal state
   */
  public void addBall() {
    balls++;
  }

  /**
   * Removes a ball from the top of the delivery internal state
   */
  public void removeBall() {
    balls--;
  }

  /**
   * @return The number of balls
   */
  public int getNumberOfBalls() {
    return balls;
  }


  //////////////////////////////////
  // ---------------------------- //
  // --- COLOR SENSOR GETTERS --- //
  // ---------------------------- //
  //////////////////////////////////

  /**
   * Corresponds to ball position 3
   * @return Whether or not the left sensor sees a ball.
   */
  public boolean getLeftColorSensorStatus() {
    return colorSensors.leftSensorSeesBall();
  }

  /**
   * Corresponds to ball position 1
   * @return Whether or not the right sensor sees a ball.
   */
  public boolean getRightColorSensorStatus() {
    return colorSensors.rightSensorSeesBall();
  }

  /**
   * Corresponds to ball position 3
   * @return The color the left sensor sees
   */
  public BallColor getLeftColorSensorValue() {
    return colorSensors.getLeftSensorBallColor();
  }

  /**
   * Corresponds to ball position 1
   * @return The color the right sensor sees.
   */
  public BallColor getRightColorSensorValue() {
    return colorSensors.getRightSensorBallColor();
  }


  //////////////////////////////////
  // ---------------------------- //
  // --- OTHER SENSOR GETTERS --- //
  // ---------------------------- //
  //////////////////////////////////

  public boolean isBallInTopSlot() {
    return getCenteringSensorStatus();
  }

  /**
   * @return Gets whether or not the centering sensor diffuse mode sensor sees something
   */
  public boolean getCenteringSensorStatus() {
    return !ballCenteringSensor.get();
  }

   /**
   * Stops the delivery mechanism
   */
  public void stopDelivery() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }

}
