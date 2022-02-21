package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.Status;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BallColor;
import frc.robot.nerdyfiles.utilities.CTREUtils;
import frc.robot.subsystems.hardware.ColorSensorTCS;

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
  private Direction direction = Direction.COUNTER_CLOCKWISE;
  private double previousDistance;
  private double tofBallCenteredDistance = 4; //Distance when we see the center of the ball
  private double tofSeesBall = 8; //Distance that we first see the edge of the ball

  // Motor
  private final TalonFX motor = new TalonFX(Constants.DELIVERY_MOTOR_ID);
  
  // Color sensors
  private final ColorSensorREV rightSensor = new ColorSensorREV(I2C.Port.kOnboard);
  private final ColorSensorTCS leftSensor = new ColorSensorTCS(I2C.Port.kMXP);
  private final TimeOfFlight tOfFlight = new TimeOfFlight(0);

  // Golf ball sensors
  private final DigitalInput topLeftBeam = new DigitalInput(Constants.TOP_LEFT_BEAM_ID);
  private final DigitalInput topRightBeam = new DigitalInput(Constants.TOP_RIGHT_BEAM_ID);
  private final DigitalInput shooterBeam = new DigitalInput(Constants.SHOOTER_BEAM_ID);


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

  /**
   * Initializes the Delivery subsystem with its two color sensors
   */
  public Delivery() {
    motor.configFactoryDefault();

    motor.setInverted(false);
    motor.setNeutralMode(NeutralMode.Brake);

    motor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit(), 0);

    setupShuffleboard(Constants.DashboardLogging.DELIVERYLOG);
  }

  private void setupShuffleboard(Boolean logEnable) {
    if (logEnable) {
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
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("TOF Distance", getDistanceInches());
  }

  ///////////////////////////
  // --------------------- //
  // --- MOTOR METHODS --- //
  // --------------------- //
  ///////////////////////////

  public void startDelivery(Direction direction) {
    startDelivery(direction, Constants.DELIVERY_SPEED);
  }

  public void startDelivery(Direction direction, double speed) {
    switch (direction) {
      case COUNTER_CLOCKWISE:
        // Rotate motor CCW
        motor.set(ControlMode.PercentOutput, speed);
        break;
      case CLOCKWISE:
        // Rotate motor CW
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
    direction = storedBalls[Slot.LEFT.value] == null ? Direction.COUNTER_CLOCKWISE : Direction.CLOCKWISE;
    return direction;
  }

  /**
   * @param ballColor The color to look for
   * @return Which way to turn in SideToTopCommand. Returns null if no need to turn or ball is on bottom.
   */
  public Direction getSideToTopDirection(BallColor ballColor) {
    if (storedBalls[Slot.LEFT.value] == ballColor) {
      // Ball is on the left, rotate clockwise
      direction = Direction.CLOCKWISE;
      return direction;
    } else if (storedBalls[Slot.RIGHT.value] == ballColor) {
      // Ball is on the right, rotate counter-clockwise
      direction = Direction.COUNTER_CLOCKWISE;
      return direction;
    } else {
      return null;
    }
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

  public Direction getDirection(){
    return direction;
  }

  /**
   * @return If there are consistency issues with the balls in the robot (if one is missing)
   */
  public boolean hasIssues() {
    int count =
      (storedBalls[Slot.BOTTOM.value] != null ? 1 : 0) +
      (storedBalls[Slot.BOTTOM.value] != null ? 1 : 0) +
      (storedBalls[Slot.BOTTOM.value] != null ? 1 : 0) +
      (storedBalls[Slot.BOTTOM.value] != null ? 1 : 0);
    
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
    return leftSensor.seesBall();
  }
  
  /**
   * Corresponds to ball position 1
   * @return Whether or not the right sensor sees a ball.
   */
  public boolean getRightColorSensorStatus() {
    return rightSensor.seesBall();
  }

  /**
   * Corresponds to ball position 3
   * @return The color the left sensor sees
   */
  public BallColor getLeftColorSensorValue() {
    return leftSensor.getColor();
  }

  /**
   * Corresponds to ball position 1
   * @return The color the right sensor sees.
   */
  public BallColor getRightColorSensorValue() {
    return rightSensor.getColor();
  }

  /**
   * @return Whether or not the ball up top is lined up
   */
  public boolean isBallLinedUpToShooter() {
    return getTopLeftSensorStatus() && getTopRightSensorStatus();
  }


  //////////////////////////////////////
  // -------------------------------- //
  // --- GOLF BALL SENSOR GETTERS --- //
  // -------------------------------- //
  //////////////////////////////////////

  /**
   * @return Gets whether or not the top left golf ball sensor sees something
   */
  public boolean getTopLeftSensorStatus() {
    return !topLeftBeam.get();
  }

  /**
   * @return Gets whether or not the top right golf ball sensor sees something
   */
  public boolean getTopRightSensorStatus() {
    return !topRightBeam.get();
  }

  /**
   * @return Gets whether or not the shooter (output) golf ball sensor sees something
   */
  public boolean getShooterSensorStatus() {
    return !shooterBeam.get();
  }

  public Status getStatus() {
    return tOfFlight.getStatus();
  }

  public double getDistanceMM() {
    return tOfFlight.getRange();
  }
  
  public double getDistanceInches() {
    if (getStatus() == Status.Valid){
      double distance = Units.metersToInches((getDistanceMM() / 1000));
      previousDistance = distance;
      return distance;
    }
    else {
      return previousDistance;
    }
  }

  public boolean tofSeesBall(){
    return (getDistanceInches() < tofSeesBall);
  }

  public boolean tofBallCentered() {
    return (getDistanceInches() < tofBallCenteredDistance);
  }
}
