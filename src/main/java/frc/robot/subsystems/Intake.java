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

  private final TalonFX firstStage;
  private final TalonFX secondStage;
  
  // Beam break sensor
  // TODO: figure out actual slots in DIO for this
  private final DigitalInput intakeBeam = new DigitalInput(0);
  
  public Intake() {
    // Initialize motor
    firstStage = new TalonFX(Constants.INTAKE_FIRST_MOTOR_ID);
    secondStage = new TalonFX(Constants.INTAKE_SECOND_MOTOR_ID);
    
    // TODO: make sure config settings are correct
    //Set settings on motor
    firstStage.configFactoryDefault();
    secondStage.configFactoryDefault();

    firstStage.setInverted(false); //TODO: make sure inversions are correct
    firstStage.setNeutralMode(NeutralMode.Coast);
    secondStage.setInverted(false);
    secondStage.setNeutralMode(NeutralMode.Coast);

    firstStage.configOpenloopRamp(0.5);
    secondStage.configOpenloopRamp(0.5);

    // Set up shuffleboard stuff
    ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
    
    ShuffleboardLayout intakeWidget = intakeTab.getLayout("Intake Info", BuiltInLayouts.kList).withSize(3,2).withPosition(4, 0);
    intakeWidget.addDoubleArray("Speed (%)", this::getIntakeSpeeds);
    intakeWidget.addDoubleArray("Temperatures (F)", this::getIntakeTemperatures);
  }

  @Override
  public void periodic() {}

  /**
   * Starts the first stage of the intake
   */
  public void startFirstStage() {
    startMotor(firstStage, Constants.INTAKE_SPEED);
  }

  /**
   * Reverses the first stage of the intake
   */
  public void reverseFirstStage() {
    startMotor(firstStage, -Constants.INTAKE_SPEED);
  }

  /**
   * Starts the second stage of the intake
   */
  public void startSecondStage() {
    startMotor(secondStage, Constants.INTAKE_SPEED);
  }

  /**
   * Reverses the second stage of the intake
   */
  public void reverseSecondStage() {
    startMotor(secondStage, -Constants.INTAKE_SPEED);
  }

  /**
   * Sets the speed of a certain motor.
   * @param motor The motor to set the speed of
   * @param speed The speed (as a percent) to set
   */
  private void startMotor(TalonFX motor, double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Calls <code>startFirstStage()</code> and <code>startSecondStage()</code> simultaneously
   */
  public void startIntake() {
    startFirstStage();
    startSecondStage();
  }

  /**
   * Reverses the intakes
   */
  public void reverseIntake() {
    reverseFirstStage();
    reverseSecondStage();
  }

  public void idleIntake() {
    startMotor(firstStage, -0.2);
    startMotor(secondStage, 0.0);
  }

  /**
   * @return Gets the intake speed as a percent (between -1 and 1)
   */
  private double[] getIntakeSpeeds() {
    return new double[]{
      Utilities.convertCelsiusToFahrenheit(firstStage.getMotorOutputPercent()),
      Utilities.convertCelsiusToFahrenheit(secondStage.getMotorOutputPercent())
    };
  }

  /**
   * Returns the temperature of the intake motor (in Celsius)
   */
  private double[] getIntakeTemperatures() {
    return new double[]{
      firstStage.getTemperature(),
      secondStage.getTemperature()
    };
  }
  
  /**
   * @return Gets whether or not the intake golf ball sensor sees something
   */
  public boolean getIntakeSensorStatus() {
    return intakeBeam.get();
  }

}
