package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem for the intake mechanism
 * 
 * @author Alex C, Michael F, Nicholas S
 */
public class Intake extends SubsystemBase {

  private final TalonFX motor;
  
  public Intake() {
    // Initialize motor
    motor = new TalonFX(Constants.INTAKE_MOTOR_ID);
    
    // TODO: make sure config settings are correct
    //Set settings on motor
    motor.configFactoryDefault();

    motor.setInverted(false); //TODO: make sure this is correct
    motor.setNeutralMode(NeutralMode.Coast);

    motor.configOpenloopRamp(0.5);

    // Set up shuffleboard stuff
    ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
    
    ShuffleboardLayout intakeWidget = intakeTab.getLayout("Intake Info", BuiltInLayouts.kList).withSize(3,2).withPosition(4, 0);
    intakeWidget.addNumber("Speed", this::getIntakeSpeed);
    intakeWidget.addNumber("Temp", this::getIntakeTemperature);
  }

  @Override
  public void periodic() {}

  /**
   * Starts the intake
   */
  public void startIntake() {
    motor.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
  }
  
  /**
   * Reverses the intake. This is the state it will be in unless we are intaking something.
   * It is equivalent to an idle state.
   */
  public void idleIntake() {
    motor.set(ControlMode.PercentOutput, -0.2);
  }

  /**
   * @return Gets the intake speed as a percent (between -1 and 1)
   */
  private double getIntakeSpeed() {
    return motor.getMotorOutputPercent();
  }

  /**
   * Returns the temperature of the intake motor (in Celsius)
   */
  private double getIntakeTemperature() {
    return motor.getTemperature();
  }

}
