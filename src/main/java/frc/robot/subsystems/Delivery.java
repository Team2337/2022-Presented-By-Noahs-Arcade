package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BallColor;

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

  // Motor
  private final TalonFX motor;
  
  // Color sensors
  private final ColorSensor leftSensor = new ColorSensor(I2C.Port.kOnboard);
  private final ColorSensor rightSensor = new ColorSensor(I2C.Port.kMXP);

  // Golf ball sensors
  // TODO: figure out actual slots in DIO for these
  private final DigitalInput intakeBeam = new DigitalInput(0);
  private final DigitalInput topLeftBeam = new DigitalInput(1);
  private final DigitalInput topRightBeam = new DigitalInput(2);
  private final DigitalInput shooterBeam = new DigitalInput(3);

  /**
   * Stores the currently held colors by rotating in a counter-clockwise direction:
   * <ul>
   * <li><code>[0]</code>: Bottom
   * <li><code>[1]</code>: Right
   * <li><code>[2]</code>: Top
   * <li><code>[3]</code>: Left
   */
  private final BallColor[] storedBalls = new BallColor[4];

  public int balls = 0;

  /**
   * Initializes the Delivery subsystem with its two color sensors
   */
  public Delivery() {
    // Initialize motor
    motor = new TalonFX(Constants.DELIVERY_MOTOR_ID);
    
    motor.configFactoryDefault();

    motor.setInverted(false); //TODO: see what actual inversion should be
    motor.setNeutralMode(NeutralMode.Brake);

    // Set up shuffleboard stuff
    ShuffleboardTab deliveryTab = Shuffleboard.getTab("Delivery");

    ShuffleboardLayout infoWidget = deliveryTab.getLayout("Info", BuiltInLayouts.kList)
      .withSize(6, 8)
      .withPosition(0, 0);
    infoWidget.addNumber("Speed (%)", () -> motor.getMotorOutputPercent());
    infoWidget.addNumber("Temperature (C)", () -> motor.getTemperature());

    ShuffleboardLayout storedLayout = deliveryTab.getLayout("Sensors", BuiltInLayouts.kList)
      .withSize(6, 8)
      .withPosition(6, 0);
    storedLayout.addStringArray("Ball positions", () -> new String[]{
      "Bottom: " + String.valueOf(storedBalls[0]),
      "Right: "  + String.valueOf(storedBalls[1]),
      "Top: "    + String.valueOf(storedBalls[2]),
      "Left: "   + String.valueOf(storedBalls[3])
    });
    storedLayout.addStringArray("Color sensors", () -> new String[]{
      "Left: "  + String.valueOf(leftSensor.getColor()),
      "Right: " + String.valueOf(rightSensor.getColor())
    });
    storedLayout.addStringArray("Golf ball sensors", () -> new String[]{
      "Intake: "    + intakeBeam.get(),
      "Top Left: "  + topLeftBeam.get(),
      "Top Right: " + topRightBeam.get(),
      "Shooter: "   + shooterBeam.get()
    });
  }

  @Override
  public void periodic() {}


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


  ////////////////////////////////////
  // ------------------------------ //
  // --- ORGANIZATIONAL METHODS --- //
  // ------------------------------ //
  ////////////////////////////////////

  public void addNewBall() {
    balls++;
    storedBalls[0] = BallColor.UNKNOWN;
  }

  public void rotateArrayClockwise() {
    storedBalls[0] = storedBalls[1];
    storedBalls[2] = storedBalls[3];
    storedBalls[1] = getRightColorSensorValue(); // 1 is right
    storedBalls[3] = getLeftColorSensorValue();  // 3 is left
  }

  public void rotateArrayCounterClockwise() {
    storedBalls[0] = storedBalls[3];
    storedBalls[2] = storedBalls[1];
    storedBalls[1] = getRightColorSensorValue(); // 1 is right
    storedBalls[3] = getLeftColorSensorValue();  // 3 is left
  }

  public Direction getCheckRotation() {
    if (storedBalls[0] == null) {
      return null;
    }

    return storedBalls[3] == null ? Direction.COUNTER_CLOCKWISE : Direction.CLOCKWISE;
  }

  public Direction getChamberDirection(BallColor ballColor) {
    if (storedBalls[3] == ballColor) {
      // Ball is on the left, rotate clockwise
      return Direction.CLOCKWISE;
    } else if (storedBalls[1] == ballColor) {
      // Ball is on the right, rotate counter-clockwise
      return Direction.COUNTER_CLOCKWISE;
    } else {
      // TODO: figure out what to do if ball *isn't* there OR if the ball is in the bottom
      return null;
    }
  }

  public BallColor getBottomPositionColor() {
    return storedBalls[0];
  }

  public BallColor getRightPositionColor() {
    return storedBalls[1];
  }

  public BallColor getTopPositionColor() {
    return storedBalls[2];
  }

  public BallColor getLeftPositionColor() {
    return storedBalls[3];
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
   * @return Gets whether or not the intake golf ball sensor sees something
   */
  public boolean getIntakeSensorStatus() {
    return intakeBeam.get();
  }

  /**
   * @return Gets whether or not the top left golf ball sensor sees something
   */
  public boolean getTopLeftSensorStatus() {
    return topLeftBeam.get();
  }

  /**
   * @return Gets whether or not the top right golf ball sensor sees something
   */
  public boolean getTopRightSensorStatus() {
    return topRightBeam.get();
  }

  /**
   * @return Gets whether or not the shooter (output) golf ball sensor sees something
   */
  public boolean getShooterSensorStatus() {
    return shooterBeam.get();
  }

}
