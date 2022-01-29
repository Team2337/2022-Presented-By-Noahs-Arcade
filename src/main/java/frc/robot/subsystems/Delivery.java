package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem for the delivery mechanism
 * 
 * @author Michael F, Alex C
 */
public class Delivery extends SubsystemBase {

  private final TalonFX motor;

  public Delivery() {
    // Initializes motor
    motor = new TalonFX(Constants.DELIVERY_MOTOR_ID);
    
    // TODO: make sure config settings are correct
    //Set settings on motor
    motor.configFactoryDefault();

    motor.setInverted(false); //TODO: make sure this is correct
    motor.setNeutralMode(NeutralMode.Coast);

    // Set up shuffleboard stuff
    ShuffleboardTab deliveryTab = Shuffleboard.getTab("Delivery");

    ShuffleboardLayout deliveryWidget = deliveryTab.getLayout("Delivery Info", BuiltInLayouts.kList).withSize(4,8).withPosition(0, 0);
    deliveryWidget.addNumber("Speed", this::getDeliverySpeed);
    deliveryWidget.addNumber("Temp", this::getDeliveryTemperature);
  }

  @Override
  public void periodic() {}

  /**
   * Starts the delivery mechanism
   */
  public void startDelivery() {
    motor.set(ControlMode.PercentOutput, Constants.DELIVERY_SPEED);
  }
  
  /**
   * Stops the delivery mechanism
   */
  public void stopDelivery() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }

  /**
   * @return Gets the delivery speed as a percent (between -1 and 1)
   */
  private double getDeliverySpeed() {
    return motor.getMotorOutputPercent();
  }

  /**
   * Returns the temperature of the delivery motor (in Celsius)
   */
  private double getDeliveryTemperature() {
    return motor.getTemperature();
  }

}
