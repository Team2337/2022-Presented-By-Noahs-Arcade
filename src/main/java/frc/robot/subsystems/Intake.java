package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriverDashboardPositions;
import frc.robot.Constants.SystemsCheckPositions;
import frc.robot.nerdyfiles.utilities.CTREUtils;

/**
 * Subsystem for the intake mechanism
 *
 * @author Alex C, Michael F, Nicholas S
 */
public class Intake extends SubsystemBase {

  private final TalonFX motor = new TalonFX(
    Constants.INTAKE_MOTOR_ID,
    Constants.UPPER_CANIVORE_ID
  );

  // Beam break sensor
  private final DigitalInput intakeBeam = new DigitalInput(Constants.getInstance().INTAKE_BEAM_ID);

  public Intake() {
    motor.configFactoryDefault();

    motor.setNeutralMode(NeutralMode.Coast);
    motor.configOpenloopRamp(0.5);

    motor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit(), 0);

    setupShuffleboard(Constants.DashboardLogging.INTAKE);
  }

  private void setupShuffleboard(Boolean logEnable) {
    // Normal debug
    if (logEnable) {
      ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");

      ShuffleboardLayout intakeWidget = intakeTab.getLayout("Intake Info", BuiltInLayouts.kList).withSize(3,2).withPosition(4, 0);
      intakeWidget.addNumber("Speed (%)", this::getSpeed);
      intakeWidget.addNumber("Temperatures (F)", this::getTemperature);
    }

    // Driver dashboard
    Constants.DRIVER_DASHBOARD.addBoolean("Intake Sensor", this::getBeamBreakSensorStatus)
      .withPosition(DriverDashboardPositions.INTAKE_BEAM.x, DriverDashboardPositions.INTAKE_BEAM.y)
      .withSize(DriverDashboardPositions.INTAKE_BEAM.width, DriverDashboardPositions.INTAKE_BEAM.height);

    // Systems check
    if (Constants.DO_SYSTEMS_CHECK) {
      ShuffleboardTab systemsCheck = Constants.SYSTEMS_CHECK_TAB;

      systemsCheck.addNumber("Intake Temp (°C)", () -> getTemperature())
        .withPosition(SystemsCheckPositions.INTAKE_TEMP.x, SystemsCheckPositions.INTAKE_TEMP.y)
        .withSize(3, 4)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", Constants.MOTOR_MINIMUM_TEMP_CELSIUS, "Max", Constants.MOTOR_SHUTDOWN_TEMP_CELSIUS));
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
  public void start() {
    setSpeed(Constants.INTAKE_FORWARD_SPEED);
  }

  /**
   * Reverses the intakes
   */
  public void reverse() {
    setSpeed(Constants.INTAKE_REVERSE_SPEED);
  }

  /**
   * Stops the intake
   */
  public void stop() {
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
    return motor.getTemperature();
  }

  /**
   * @return Gets whether or not the intake golf ball sensor sees something
   */
  public boolean getBeamBreakSensorStatus() {
    return intakeBeam.get();
  }

}
