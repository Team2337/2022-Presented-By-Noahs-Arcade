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
import frc.robot.nerdyfiles.utilities.CTREUtils;
import frc.robot.nerdyfiles.utilities.Utilities;

/**
 * Subsystem for the intake mechanism
 *
 * @author Alex C, Michael F, Nicholas S
 */
public class Intake extends SubsystemBase {

  // Motor
  private final TalonFX motor = new TalonFX(Constants.INTAKE_MOTOR_ID);

  // Beam break sensor
  private final DigitalInput intakeBeam = new DigitalInput(Constants.INTAKE_SENSOR_ID);

  public Intake() {
    motor.configFactoryDefault();

    motor.setNeutralMode(NeutralMode.Coast);
    motor.configOpenloopRamp(0.5);

    motor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit(), 0);

    setupShuffleboard(Constants.DashboardLogging.INTAKELOG);
  }

  private void setupShuffleboard(Boolean logEnable) {
    if (logEnable) {
      ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");

      ShuffleboardLayout intakeWidget = intakeTab.getLayout("Intake Info", BuiltInLayouts.kList).withSize(3,2).withPosition(4, 0);
      intakeWidget.addNumber("Speed (%)", this::getSpeed);
      intakeWidget.addNumber("Temperatures (F)", this::getTemperature);
    }
  }

  @Override
  public void periodic() {}

  /**
   * Sets the intake speed
   * @param speed The speed (as a percent, -1.0 to 1.0)
   */
  private void setSpeed(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Starts the intake motor
   */
  public void startIntake() {
    setSpeed(Constants.INTAKE_FORWARD_SPEED);
  }

  /**
   * Reverses the intakes
   */
  public void reverseIntake() {
    setSpeed(Constants.INTAKE_REVERSE_SPEED);
  }

  /**
   * Stops the intake
   */
  public void stopIntake() {
    setSpeed(0.0);
  }

  /**
   * @return Gets the intake speed as a percent (between -1 and 1)
   */
  private double getSpeed() {
    return motor.getMotorOutputPercent();
  }

  /**
   * Returns the temperature of the intake motor (in Celsius)
   */
  private double getTemperature() {
    return Utilities.convertCelsiusToFahrenheit(motor.getTemperature());
  }

  /**
   * @return Gets whether or not the intake golf ball sensor sees something
   */
  public boolean getIntakeSensorStatus() {
    return !intakeBeam.get();
  }

}
