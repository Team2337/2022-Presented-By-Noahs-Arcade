package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem for the delivery mechanism
 * 
 * @author Michael F, Alex C
 */
public class Delivery extends SubsystemBase {

  private final ShuffleboardTab tab = Shuffleboard.getTab("Delivery");

  private final TalonFX motor;

  public Delivery() {
    // Initializes motor
    motor = new TalonFX(Constants.MODULE3_DRIVE_MOTOR_ID);
    
    // TODO: make sure config settings are correct
    //Set settings on motor
    motor.configFactoryDefault();

    motor.setInverted(false); //TODO: make sure this is correct
    motor.setNeutralMode(NeutralMode.Coast);

    //Configure a current limit
    StatorCurrentLimitConfiguration intakeCurrentLimitConfig = 
      new StatorCurrentLimitConfiguration();
    intakeCurrentLimitConfig.currentLimit = 50;
    intakeCurrentLimitConfig.enable = true;
    intakeCurrentLimitConfig.triggerThresholdCurrent = 40;
    intakeCurrentLimitConfig.triggerThresholdTime = 3;
    //Push the current limit to the motor
    motor.configStatorCurrentLimit(intakeCurrentLimitConfig, 0);

    //Configure motor ramp rate
    motor.configClosedloopRamp(0.5);

    // Set up shuffleboard stuff
    ShuffleboardLayout intakeWidget = tab.getLayout("Delivery Info", BuiltInLayouts.kList).withSize(3,2).withPosition(11, 16);
    intakeWidget.addNumber("Speed", () -> getDeliverySpeed());
    intakeWidget.addNumber("Temp", () -> getDeliveryTemperature());
  }

  @Override
  public void periodic() {
    // Put debug information on SmartDashboard
    // SmartDashboard.putNumber("delivery/speed", getDeliverySpeed());
    // SmartDashboard.putNumber("delivery/temperature", getDeliveryTemperature());
  }

  /**
   * Sets the intake speed
   * @param speed The speed as a percent (from -1 to 1)
   */
  public void setDeliverySpeed(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }
  
  /**
   * Stops the intake
   */
  public void stopDelivery() {
    setDeliverySpeed(0.);
  }

  /**
   * @return Gets the intake speed as a percent (between -1 and 1)
   */
  public double getDeliverySpeed() {
    return motor.getMotorOutputPercent();
  }

  /**
   * Returns the temperature of the intake motor (in Celsius)
   */
  public double getDeliveryTemperature() {
    return motor.getTemperature();
  }

}
