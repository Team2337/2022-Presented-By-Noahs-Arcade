package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem for the intake mechanism
 * 
 * @author Alex C, Michael F, Nicholas S
 */
public class Intake extends SubsystemBase {

  private final TalonFX intakeMotor;
  
  /**
   * 
   */
  public Intake() {
    intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID);
  }

  @Override
  public void periodic() {
    // Put debug information on SmartDashboard
    SmartDashboard.putNumber("intake/speed", getIntakeSpeed());
    SmartDashboard.putNumber("intake/temperature", getIntakeTemperature());
  }

  /**
   * Sets the intake speed
   * @param speed The speed as a percent (from -1 to 1)
   */
  public void setIntakeSpeed(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }
  
  /**
   * Stops the intake
   */
  public void stopIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return Gets the intake speed as a percent (between -1 and 1)
   */
  public double getIntakeSpeed(){
    return intakeMotor.getMotorOutputPercent();
  }

  /**
   * Returns the temperature of the intake motor (in Celsius)
   */
  public double getIntakeTemperature(){
    return intakeMotor.getTemperature();
  }

}
