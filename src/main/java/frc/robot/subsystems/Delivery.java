package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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

  private static enum Slot {
    BOTTOM(0),
    RIGHT(1),
    TOP(2),
    LEFT(3);

    public final int value;

    private Slot(int value) {
      this.value = value;
    }
  }

  private final TalonFX motor = new TalonFX(
    Constants.DELIVERY_MOTOR_ID,
    Constants.UPPER_CANIVORE_ID
  );

  // Color sensors
  private final PicoColorSensors colorSensors = new PicoColorSensors();
  
  private final DigitalInput ballCenteringSensor = new DigitalInput(Constants.getInstance().CENTERING_BEAM_ID);


  /**
   * Stores the currently held colors by rotating in a counter-clockwise direction:
   * <ul>
   * <li><code>[0]</code>: Bottom
   * <li><code>[1]</code>: Right
   * <li><code>[2]</code>: Top
   * <li><code>[3]</code>: Left
   */
  private final BallColor[] storedBalls = new BallColor[4];

  private int balls = 0;

  private String alliance;

  /**
   * Initializes the Delivery subsystem.
   */
  public Delivery() {
    motor.configFactoryDefault();

    motor.setNeutralMode(NeutralMode.Brake);
    motor.configOpenloopRamp(0.5);

    motor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit(), 0);

    setupShuffleboard(Constants.DashboardLogging.DELIVERY);

    switch (DriverStation.getAlliance()) {
      default:
      case Red:
        // If alliance is red, prioritize red targets if they are there;
        // otherwise get blue targets
        alliance = "Red";
        break;
      case Blue:
        // If alliance is blue, prioritize blue targets if they are there;
        // otherwise get red targets
        alliance = "Blue";
        break;
      }
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
      sensorsWidget.addStringArray("Ball positions", () -> new String[]{
        "Bottom: " + String.valueOf(storedBalls[Slot.BOTTOM.value]),
        "Right: "  + String.valueOf(storedBalls[Slot.RIGHT.value]),
        "Top: "    + String.valueOf(storedBalls[Slot.TOP.value]),
        "Left: "   + String.valueOf(storedBalls[Slot.LEFT.value])
      });
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
      /*
      systemsCheck.addBoolean("Left Color Sensor", () -> colorSensors.leftSensorIsConnected())
        .withPosition(SystemsCheckPositions.L_COLOR_SENSOR.x, SystemsCheckPositions.L_COLOR_SENSOR.y)
        .withSize(3, 3);
      systemsCheck.addBoolean("Right Color Sensor", () -> colorSensors.rightSensorIsConnected())
        .withPosition(SystemsCheckPositions.R_COLOR_SENSOR.x, SystemsCheckPositions.R_COLOR_SENSOR.y)
        .withSize(3, 3);
      */
      systemsCheck.addNumber("Delivery Temp (C)", () -> getTemperature())
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
    SmartDashboard.putString("Left Ball Color", getLeftColorSensorAllianceBallColor());
    SmartDashboard.putString("Left Color Raw", colorSensors.getColor0());
    SmartDashboard.putString("Right Color Raw", colorSensors.getColor1());
    int val = colorSensors.getLeftSensorBallColor() == BallColor.Red ? 1 : (colorSensors.getLeftSensorBallColor() == BallColor.Blue ? 2 : 0);
    SmartDashboard.putNumber("Left Color Graph", val);
  }


  ///////////////////////////
  // --------------------- //
  // --- MOTOR METHODS --- //
  // --------------------- //
  ///////////////////////////

  public void start(Direction direction) {
    setSpeed(direction, Constants.DELIVERY_SPEED);
  }

  public void start(double speed, Direction direction) {
    setSpeed(direction, speed);
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
  public void addNewBall() {
    balls++;
    storedBalls[Slot.BOTTOM.value] = BallColor.UNKNOWN;
  }

  /**
   * Removes a ball from the top of the delivery internal state
   */
  public void removeTopBall() {
    balls--;
    storedBalls[Slot.TOP.value] = null;
  }

  /**
   * Rotates the internal state clockwise
   */
  public void rotateArrayClockwise() {
    /*
    storedBalls[Slot.BOTTOM.value] = storedBalls[Slot.RIGHT.value];
    storedBalls[Slot.RIGHT.value] = storedBalls[Slot.TOP.value];
    storedBalls[Slot.TOP.value] = storedBalls[Slot.LEFT.value];
    storedBalls[Slot.LEFT.value] = getLeftColorSensorValue();
    */
  }

  /**
   * Rotates the internal state counter-clockwise
   */
  public void rotateArrayCounterClockwise() {
    /*
    storedBalls[Slot.BOTTOM.value] = storedBalls[Slot.LEFT.value];
    storedBalls[Slot.LEFT.value] = storedBalls[Slot.TOP.value];
    storedBalls[Slot.TOP.value] = storedBalls[Slot.RIGHT.value];
    storedBalls[Slot.RIGHT.value] = getRightColorSensorValue();
    */
  }

  public void resetArray() {
    /*
    storedBalls[Slot.BOTTOM.value] = null;
    storedBalls[Slot.RIGHT.value] = getRightColorSensorValue();
    storedBalls[Slot.TOP.value] = null;
    storedBalls[Slot.LEFT.value] = getLeftColorSensorValue();
    */
  }

  /**
   * @return Which way to turn in BottomToSideCommand
   */
  public Direction getBottomToSideRotation() {
    if (storedBalls[Slot.BOTTOM.value] == null) {
      return null;
    }

    return storedBalls[Slot.RIGHT.value] == null ? Direction.CLOCKWISE : Direction.COUNTER_CLOCKWISE;
  }

  /**
   * @param ballColor The color to look for
   * @return Which way to turn in SideToTopCommand. Returns null if no need to turn or ball is on bottom.
   */
  public Direction getSideToTopDirection(BallColor ballColor) {
    if (storedBalls[Slot.LEFT.value] == ballColor) {
      // Ball is on the left, rotate clockwise
      return Direction.CLOCKWISE;
    } else if (storedBalls[Slot.RIGHT.value] == ballColor) {
      // Ball is on the right, rotate counter-clockwise
      return Direction.COUNTER_CLOCKWISE;
    }
    return null;
  }

  /**
   * @return The color of the bottom slot of the internal state
   */
  public BallColor getBottomPositionColor() {
    return storedBalls[Slot.BOTTOM.value];
  }

  /**
   * @return The color of the right slot of the internal state
   */
  public BallColor getRightPositionColor() {
    return storedBalls[Slot.RIGHT.value];
  }

  /**
   * @return The color of the top slot of the internal state
   */
  public BallColor getTopPositionColor() {
    return storedBalls[Slot.TOP.value];
  }

  /**
   * @return The color of the left slot of the internal state
   */
  public BallColor getLeftPositionColor() {
    return storedBalls[Slot.LEFT.value];
  }

  /**
   * @return The number of balls
   */
  public int getNumberOfBalls() {
    return balls;
  }

  /**
   * @return If there are consistency issues with the balls in the robot (if one is missing)
   */
  public boolean hasIssues() {
    int count =
      (storedBalls[Slot.BOTTOM.value] != null ? 1 : 0) +
      (storedBalls[Slot.LEFT.value] != null ? 1 : 0) +
      (storedBalls[Slot.RIGHT.value] != null ? 1 : 0) +
      (storedBalls[Slot.TOP.value] != null ? 1 : 0);

    // If count == balls, return false because there are no issues. Otherwise return true.
    return count != balls;
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

  public String getLeftColorSensorAllianceBallColor() {
    BallColor incoming = colorSensors.getLeftSensorBallColor();
    if (incoming != null) {
      return String.valueOf(incoming);
    } else {
      return "null";
    }
  }

  public String getRightColorSensorAllianceBallColor() {
    BallColor incoming = colorSensors.getRightSensorBallColor();
    if (incoming != null) {
      return String.valueOf(incoming);
    } else {
      return "null";
    }
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
