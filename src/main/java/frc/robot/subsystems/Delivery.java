package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BallColor;

/**
 * Subsystem for the delivery mechanism
 * 
 * @author Michael F, Alex C
 */
public class Delivery extends SubsystemBase {

  private final TalonFX motor;
  private final ColorSensor leftSensor;
  private final ColorSensor rightSensor;

  /**
   * Stores the currently held colors in this order:
   * <ul>
   * <li><code>[0]</code>: Bottom
   * <li><code>[1]</code>: Right
   * <li><code>[2]</code>: Top
   * <li><code>[3]</code>: Left
   */
  private final BallColor[] storedColors;

  private int turnsToMake = 0;
  private boolean motorsActive = false;//TODO: do we need this?

  /**
   * Initializes the Delivery subsystem with its two color sensors
   */
  public Delivery() {
    // Initialize motor
    motor = new TalonFX(Constants.DELIVERY_MOTOR_ID);
    
    motor.configFactoryDefault();

    motor.setInverted(false); //TODO: make sure this is correct
    motor.setNeutralMode(NeutralMode.Brake);

    // Initialize color sensor
    this.leftSensor = new ColorSensor(I2C.Port.kOnboard);
    this.rightSensor = new ColorSensor(I2C.Port.kMXP);

    // Initialize stored objects array
    storedColors = new BallColor[4];

    // Set up shuffleboard stuff
    ShuffleboardTab deliveryTab = Shuffleboard.getTab("Delivery");

    ShuffleboardLayout storedLayout = deliveryTab.getLayout("Stored items", BuiltInLayouts.kList)
      .withSize(6, 8)
      .withPosition(4, 0);
    storedLayout.addStringArray("title", () -> new String[]{
      "Bottom: " + storedColors[0].toString(),
      "Right: "  + storedColors[1].toString(),
      "Top: "    + storedColors[2].toString(),
      "Left: "   + storedColors[3].toString()
    });

  }

  @Override
  public void periodic() {}

  /**
   * Starts turning the delivery mechanism right
   */
  public void turnDeliveryRight() {
    motor.set(ControlMode.PercentOutput, Constants.DELIVERY_SPEED);
    motorsActive = true;
  }

  /**
   * Starts turning the delivery mechanism left
   */
  public void turnDeliveryLeft(){
    motor.set(ControlMode.PercentOutput, -Constants.DELIVERY_SPEED);
    motorsActive = true;
  }
  
  /**
   * Stops the delivery mechanism
   */
  public void stopDelivery() {
    motor.set(ControlMode.PercentOutput, 0.0);
    motorsActive = false;
  }

  // 

}
