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

/**
 * Subsystem for the intake mechanism
 * 
 * @author Alex C, Michael F, Nicholas S
 */
public class Intake extends SubsystemBase {

  private final TalonFX firstStage;
  private final TalonFX secondStage;
  
  // TODO: figure out actual slots in DIO for this
  private final DigitalInput intakeBeam = new DigitalInput(0);
  
  public Intake() {
    // Initialize motor
    firstStage = new TalonFX(Constants.INTAKE_FIRST_MOTOR_ID);
    secondStage = new TalonFX(Constants.INTAKE_FIRST_MOTOR_ID);
    
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
    
    ShuffleboardLayout intakeWidget = intakeTab.getLayout("Intake Info", BuiltInLayouts.kList)
      .withSize(3,2)
      .withPosition(4, 0);
    intakeWidget.addDoubleArray("Speed", this::getIntakeSpeeds);
    intakeWidget.addDoubleArray("Temp", this::getIntakeTemperatures);

    ShuffleboardLayout sensorsWidget = intakeTab.getLayout("Sensor", BuiltInLayouts.kList)
      .withSize(6, 8)
      .withPosition(6, 0);
    sensorsWidget.addBoolean("Intake sensor", intakeBeam::get);
  }

  @Override
  public void periodic() {}

  /**
   * Starts the first stage of the intake
   */
  public void startFirstStage() {
    firstStage.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
  }

  /**
   * Starts the second stage of the intake
   */
  public void startSecondStage() {
    secondStage.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
  }

  public void reverseFirstStage() {
    firstStage.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
  }

  /**
   * Calls <code>startFirstStage()</code> and <code>startSecondStage()</code> simultaneously
   */
  public void startIntake() {
    startFirstStage();
    startSecondStage();
  }

  // TODO: keep reverse or idle?
  /**
   * Reverses the intakes
   */
  public void reverseIntake() {
    firstStage.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
    secondStage.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
  }
  
  /**
   * Reverses the intake. This is the state it will be in unless we are intaking something.
   * It is equivalent to an idle state.
   */
  public void idleIntake() {
    firstStage.set(ControlMode.PercentOutput, -0.2);
    secondStage.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return Gets the intake speed as a percent (between -1 and 1)
   */
  private double[] getIntakeSpeeds() {
    return new double[]{
      firstStage.getMotorOutputPercent(),
      secondStage.getMotorOutputPercent()
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
