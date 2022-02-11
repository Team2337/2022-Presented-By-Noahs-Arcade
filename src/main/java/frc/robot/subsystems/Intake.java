package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities;

/**
 * Subsystem for the intake mechanism
 * 
 * @author Alex C, Michael F, Nicholas S
 */
public class Intake extends SubsystemBase {

  private final TalonFX motor = new TalonFX(Constants.INTAKE_MOTOR_ID);
  
  // Beam break sensor
  private final DigitalInput intakeBeam = new DigitalInput(Constants.DIO.INTAKE_BEAM_ID);
  
  public Intake() {
    //Set settings on motor
    motor.configFactoryDefault();

    motor.setNeutralMode(NeutralMode.Coast);

    motor.configOpenloopRamp(0.5);

    // Set up shuffleboard stuff
    ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
    
    ShuffleboardLayout intakeWidget = intakeTab.getLayout("Intake Info", BuiltInLayouts.kList)
      .withSize(3,2)
      .withPosition(4, 0);
    intakeWidget.addNumber("Speed (%)", this::getIntakeSpeeds);
    intakeWidget.addNumber("Temperatures (F)", this::getIntakeTemperatures);

    ShuffleboardLayout sensorsWidget = intakeTab.getLayout("Sensor", BuiltInLayouts.kList)
      .withSize(6, 8)
      .withPosition(6, 0);
    sensorsWidget.addBoolean("Intake sensor", intakeBeam::get);
  }

  @Override
  public void periodic() {}

  /**
   * Starts the intake motor
   */
  public void startIntake() {
    setIntakeSpeed(Constants.INTAKE_SPEED);
  }

  public void setIntakeSpeed(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Reverses the intakes
   */
  public void reverseIntake() {
    setIntakeSpeed(-Constants.INTAKE_SPEED);
  }

  public void stopIntake() {
    setIntakeSpeed(0.0);
  }

  /**
   * @return Gets the intake speed as a percent (between -1 and 1)
   */
  private double getIntakeSpeeds() {
    return motor.getMotorOutputPercent();
  }

  /**
   * Returns the temperature of the intake motor (in Celsius)
   */
  private double getIntakeTemperatures() {
    return Utilities.convertCelsiusToFahrenheit(motor.getTemperature());
  }
  
  /**
   * @return Gets whether or not the intake golf ball sensor sees something
   */
  public boolean getIntakeSensorStatus() {
    return !intakeBeam.get();
  }

}
