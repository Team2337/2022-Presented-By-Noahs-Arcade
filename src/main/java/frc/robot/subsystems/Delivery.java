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
  public final BallColor[] storedBalls = new BallColor[4];

  public int balls = 0;

  public boolean linedUp = false;

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

    ShuffleboardLayout storedLayout = deliveryTab.getLayout("Sensors", BuiltInLayouts.kList)
      .withSize(6, 8)
      .withPosition(4, 0);
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

  /**
   * Starts turning the delivery mechanism
   * @param speed The speed at which to turn as a percent
   */
  public void setDeliverySpeed(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }
  
  /**
   * Stops the delivery mechanism
   */
  public void stopDelivery() {
    motor.set(ControlMode.PercentOutput, 0.0);
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
