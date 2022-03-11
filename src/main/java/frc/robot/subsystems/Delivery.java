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
import frc.robot.subsystems.hardware.TimeOfFlightSensor;

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
  // private final PicoColorSensors colorSensors = new PicoColorSensors();

  // TOF sensor
  // private final TimeOfFlightSensor lineupSensor = new TimeOfFlightSensor();

  // Beam break sensor
  // private final DigitalInput shooterBeam = new DigitalInput(Constants.SHOOTER_BEAM_ID);

  private final DigitalInput ballCenteringSensor = new DigitalInput(Constants.CENTERING_BEAM_ID);


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
  private static final double LINEUP_SENSOR_MAX_DISTANCE_INCHES = 4.5;


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
      sensorsWidget.addStringArray("Ball positions", () -> new String[]{
        "Bottom: " + String.valueOf(storedBalls[Slot.BOTTOM.value]),
        "Right: "  + String.valueOf(storedBalls[Slot.RIGHT.value]),
        "Top: "    + String.valueOf(storedBalls[Slot.TOP.value]),
        "Left: "   + String.valueOf(storedBalls[Slot.LEFT.value])
      });
      /*
      sensorsWidget.addStringArray("Color sensors", () -> new String[]{
        "Left: " + String.valueOf(colorSensors.getLeftSensorBallColor()),
        "Right: " + String.valueOf(colorSensors.getRightSensorBallColor())
      });
      sensorsWidget.addStringArray("Proximities", () -> new String[]{
        "Left: " + String.valueOf(colorSensors.leftSensorSeesBall()),
        "Right: " + String.valueOf(colorSensors.rightSensorSeesBall())
      });
      sensorsWidget.addStringArray("Other sensors", () -> new String[]{
        "Lineup (in): "  + lineupSensor.getDistanceInches(),
        "Shooter: "   + shooterBeam.get()
      });
      */
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
      systemsCheck.addBoolean("Time Of Flight", lineupSensor::systemsCheck)
        .withPosition(SystemsCheckPositions.TOF_SENSOR.x, SystemsCheckPositions.TOF_SENSOR.y)
        .withSize(3, 3);
      */
      systemsCheck.addNumber("Delivery Temp (°C)", () -> getTemperature())
        .withPosition(SystemsCheckPositions.DELIVERY_TEMP.x, SystemsCheckPositions.DELIVERY_TEMP.y)
        .withSize(3, 4)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", Constants.MOTOR_MINIMUM_TEMP_CELSIUS, "Max", Constants.MOTOR_SHUTDOWN_TEMP_CELSIUS));
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter Sensor Status", !ballCenteringSensor.get());
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
    storedBalls[Slot.BOTTOM.value] = storedBalls[Slot.RIGHT.value];
    storedBalls[Slot.RIGHT.value] = storedBalls[Slot.TOP.value];
    storedBalls[Slot.TOP.value] = storedBalls[Slot.LEFT.value];
    storedBalls[Slot.LEFT.value] = getLeftColorSensorValue();
  }

  /**
   * Rotates the internal state counter-clockwise
   */
  public void rotateArrayCounterClockwise() {
    storedBalls[Slot.BOTTOM.value] = storedBalls[Slot.LEFT.value];
    storedBalls[Slot.LEFT.value] = storedBalls[Slot.TOP.value];
    storedBalls[Slot.TOP.value] = storedBalls[Slot.RIGHT.value];
    storedBalls[Slot.RIGHT.value] = getRightColorSensorValue();
  }

  public void resetArray() {
    storedBalls[Slot.BOTTOM.value] = null;
    storedBalls[Slot.RIGHT.value] = getRightColorSensorValue();
    storedBalls[Slot.TOP.value] = null;
    storedBalls[Slot.LEFT.value] = getLeftColorSensorValue();
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
    return false; // colorSensors.leftSensorSeesBall();
  }

  /**
   * Corresponds to ball position 1
   * @return Whether or not the right sensor sees a ball.
   */
  public boolean getRightColorSensorStatus() {
    return false; // colorSensors.rightSensorSeesBall();
  }

  /**
   * Corresponds to ball position 3
   * @return The color the left sensor sees
   */
  public BallColor getLeftColorSensorValue() {
    return null; // colorSensors.getLeftSensorBallColor();
  }

  /**
   * Corresponds to ball position 1
   * @return The color the right sensor sees.
   */
  public BallColor getRightColorSensorValue() {
    return null; // colorSensors.getRightSensorBallColor();
  }


  //////////////////////////////////
  // ---------------------------- //
  // --- OTHER SENSOR GETTERS --- //
  // ---------------------------- //
  //////////////////////////////////

  /**
   * @return The reading of the lineup time of flight sensor in inches
   */
  public double getLineupSensorValue() {
    return 0; // lineupSensor.getDistanceInches();
  }

  public boolean isBallInTopSlot() {
    // 3.5 seems to be the maximum value when a ball is lined up, it's a pretty big difference beyond that
    return getShooterSensorStatus();
  }

  /**
   * TODO: does this need to be moved to shooter when we add that logic? Will this be in the robot?
   * @return Gets whether or not the shooter (output) golf ball sensor sees something
   */
  public boolean getShooterSensorStatus() {
    return !ballCenteringSensor.get();
  }

    /**
   * Stops the delivery mechanism
   */
  public void stopDelivery() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }

}
