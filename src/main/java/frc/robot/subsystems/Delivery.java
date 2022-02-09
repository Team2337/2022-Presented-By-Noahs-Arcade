package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem for the delivery mechanism
 * 
 * @author Michael F, Nicholas S, Alex C
 */
public class Delivery extends SubsystemBase {

  // Direction enum
  public static enum Direction {
    CLOCKWISE,
    COUNTER_CLOCKWISE
  }

  // Motor
  private final TalonFX motor;

  /**
   * Initializes the Delivery subsystem with its two color sensors
   */
  public Delivery() {
    // Initialize motor
    motor = new TalonFX(Constants.DELIVERY_MOTOR_ID);

    // TODO: make sure config settings are correct
    //Set settings on motor
    motor.configFactoryDefault();

    motor.setInverted(false);
    motor.setNeutralMode(NeutralMode.Brake);

    motor.configOpenloopRamp(0.5);

    // Set up shuffleboard stuff
    ShuffleboardTab deliveryTab = Shuffleboard.getTab("Delivery");
    ShuffleboardLayout infoWidget = deliveryTab.getLayout("Info", BuiltInLayouts.kList);
    deliveryTab.addNumber("Speed", () -> motor.getMotorOutputPercent());
    deliveryTab.addNumber("Temperature (C)", () -> motor.getTemperature());
  }

  @Override
  public void periodic() {}


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
