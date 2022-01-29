package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.DigitalInput;
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

  // Motor
  private final TalonFX motor;

  // Color sensors
  public final ColorSensor leftSensor = new ColorSensor(I2C.Port.kOnboard);
  public final ColorSensor rightSensor = new ColorSensor(I2C.Port.kMXP);

  // Golf ball sensors
  // TODO: figure out actual slots in DIO for these
  public final DigitalInput bottomBeam = new DigitalInput(0);
  public final DigitalInput topLeftBeam = new DigitalInput(1);
  public final DigitalInput topRightBeam = new DigitalInput(2);
  public final DigitalInput outBeam = new DigitalInput(3);

  /**
   * Stores the currently held colors in this order:
   * <ul>
   * <li><code>[0]</code>: Bottom
   * <li><code>[1]</code>: Right
   * <li><code>[2]</code>: Top
   * <li><code>[3]</code>: Left
   */
  public final BallColor[] storedColors = new BallColor[4];

  public int balls = 0;

  /**
   * Initializes the Delivery subsystem with its two color sensors
   */
  public Delivery() {
    // Initialize motor
    motor = new TalonFX(Constants.DELIVERY_MOTOR_ID);
    
    motor.configFactoryDefault();

    motor.setInverted(false); //TODO: make sure this is correct
    motor.setNeutralMode(NeutralMode.Brake);

    // Set up shuffleboard stuff
    ShuffleboardTab deliveryTab = Shuffleboard.getTab("Delivery");

    ShuffleboardLayout storedLayout = deliveryTab.getLayout("Stored items", BuiltInLayouts.kList)
      .withSize(6, 8)
      .withPosition(4, 0);
    storedLayout.addStringArray("title", () -> new String[]{
      "Bottom: " + String.valueOf(storedColors[0]),
      "Right: "  + String.valueOf(storedColors[1]),
      "Top: "    + String.valueOf(storedColors[2]),
      "Left: "   + String.valueOf(storedColors[3])
    });

  }

  @Override
  public void periodic() {}

  /**
   * Starts turning the delivery mechanism
   */
  public void startDelivery(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }
  
  /**
   * Stops the delivery mechanism
   */
  public void stopDelivery() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }

  // 

}
