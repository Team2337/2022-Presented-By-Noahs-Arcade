package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem for the delivery mechanism
 * 
 * @author Michael F, Alex C
 */
public class Delivery extends SubsystemBase {

  private final TalonFX deliveryMotor;

  /**
   * Delivery constructor
   */
  public Delivery() {
    deliveryMotor = new TalonFX(Constants.MODULE3_DRIVE_MOTOR_ID);
  }

  @Override
  public void periodic() {
    // Put debug information on SmartDashboard
    SmartDashboard.putNumber("delivery/speed", getDeliverySpeed());
    SmartDashboard.putNumber("delivery/temperature", getDeliveryTemperature());
  }

  /**
   * Sets the intake speed
   * @param speed The speed as a percent (from -1 to 1)
   */
  public void setDeliverySpeed(double speed){
    deliveryMotor.set(ControlMode.PercentOutput, speed);
  }
  
  /**
   * Stops the intake
   */
  public void stopDelivery(){
    deliveryMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return Gets the intake speed as a percent (between -1 and 1)
   */
  public double getDeliverySpeed(){
    return deliveryMotor.getMotorOutputPercent();
  }

  /**
   * Returns the temperature of the intake motor (in Celsius)
   */
  public double getDeliveryTemperature(){
    return deliveryMotor.getTemperature();
  }

}
