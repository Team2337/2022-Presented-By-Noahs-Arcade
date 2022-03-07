package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SystemsCheckPositions;
import frc.robot.nerdyfiles.utilities.CTREUtils;

/**
 * Subsystem for the climber mechanism
 */
public class Climber extends SubsystemBase {

  private final AnalogInput stringPot = new AnalogInput(Constants.CLIMBER_STRING_POT_ID);
  private final TalonFX leftMotor = new TalonFX(Constants.CLIMBER_LEFT_MOTOR_ID);
  private final TalonFX rightMotor = new TalonFX(Constants.CLIMBER_RIGHT_MOTOR_ID);

  public Climber() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

    rightMotor.follow(leftMotor);

    leftMotor.setInverted(TalonFXInvertType.Clockwise);
    rightMotor.setInverted(TalonFXInvertType.OpposeMaster);

    leftMotor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit(), 0);
    // TODO: If set set a nominal voltage we can enable voltage compensation
    // leftMotor.enableVoltageCompensation(true);

    // PID for our positional hold
    leftMotor.config_kP(0, 0.15);
    leftMotor.config_kI(0, 0);
    leftMotor.config_kD(0, 0);
    // Motor turns 16 times for one climber rotation, which is 6.283 inches, 2048
    // ticks in a rotation. Overall loss with this tolerance: 0.019 inches
    leftMotor.configAllowableClosedloopError(0, 100);
    leftMotor.configNominalOutputForward(0.1);
    leftMotor.configNominalOutputReverse(0.1);

    setupShuffleboard(Constants.DashboardLogging.CLIMBER);
  }

  private void setupShuffleboard(Boolean logEnable) {
    if (logEnable) {
      ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
      ShuffleboardLayout climberWidget = climberTab.getLayout("climber Info", BuiltInLayouts.kList).withSize(3, 2)
          .withPosition(4, 0);
      climberWidget.addNumber("Speed", this::getSpeed);
      climberWidget.addNumber("Left Temp", this::getLeftMotorTemperature);
      climberWidget.addNumber("Right Temp", this::getRightMotorTemperature);
      climberWidget.addNumber("String Pot", this::getStringPotVoltage);
    }

    // Systems check
    ShuffleboardTab systemsCheck = Constants.SYSTEMS_CHECK_TAB;
    
    systemsCheck.addBoolean("String Pot", () -> (stringPot.getVoltage() > 0))
      .withSize(2, 2)
      .withPosition(SystemsCheckPositions.STRING_POT.x, SystemsCheckPositions.STRING_POT.y);
  }

  @Override
  public void periodic() {}

  public void hold(double position) {
    leftMotor.set(ControlMode.Position, position);
  }

  public double getPosition() {
    return leftMotor.getSelectedSensorPosition();
  }

  public void stop() {
    setSpeed(0.0);
  }

  public void setSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
  }

  private double getSpeed() {
    return leftMotor.getMotorOutputPercent();
  }

  public double getStringPotVoltage() {
    return stringPot.getVoltage();
  }

  public boolean isStringPotConnected() {
    return getStringPotVoltage() > 0;
  }

  /**
   * Temp in Celcius
   */
  private double getLeftMotorTemperature() {
    return leftMotor.getTemperature();
  }

  /**
   * Temp in Celcius
   */
  private double getRightMotorTemperature() {
    return rightMotor.getTemperature();
  }

}
