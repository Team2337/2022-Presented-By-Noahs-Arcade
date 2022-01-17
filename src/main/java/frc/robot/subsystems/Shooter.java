package frc.robot.subsystems;


import java.util.Map;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/* @nicholas.stokes If you’d like to own getting the shooter code setup
 and ready to go that would be good. Two Falcons. Make sure we have some Dashboard element pre-built using code that we can 
 configure things like the RPM for each motor, along with the PIDs (keep in mind the PIDs are going to be on the Falcon’s)
*/
public class Shooter extends SubsystemBase {

    private TalonFX leftShoot;
    private TalonFX rightShoot;
    public TalonFXConfiguration fxConfig;
    public StatorCurrentLimitConfiguration currentLimitConfigurationMotor = new StatorCurrentLimitConfiguration();

    private double kP = 1.15;
    private double kI = 0;
    private double kD = 0.0002;
    private double kF = 0;

    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private double prevLeftSpeed = 0;
    private double prevRightSpeed = 0;
    private int rightCounter = 0;
    private int leftCounter = 0;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    ShuffleboardLayout pid = tab.getLayout("PID Control", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(0, 0);
    ShuffleboardLayout speeds = tab.getLayout("Shooter Speeds", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(2, 0); 
     public NetworkTableEntry kep = pid
        .add("kP", kP )
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();    
     public NetworkTableEntry kei = pid
        .add("kI", kI )
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();   
 
     public NetworkTableEntry ked = pid
        .add("kD", kD )
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();    
 
     public NetworkTableEntry kef = pid
        .add("kF", kF )
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();  

     public NetworkTableEntry leftShooter = speeds
        .add("Left Shooter Speed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 100))
        .getEntry();
    public NetworkTableEntry rightShooter = speeds
        .add("Right Shooter Speed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 100))
        .getEntry();

    public Shooter() {
        leftShoot = new TalonFX(Constants.SHOOTER_LEFT_MOTOR);
        rightShoot = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR);
        fxConfig = new TalonFXConfiguration();
        

        /** --- CONFIGURE MOTOR AND SENSOR SETTINGS --- **/
        // Configures motors to factory default
        leftShoot.configFactoryDefault();
        rightShoot.configFactoryDefault();
        // Configures sensors for PID calculations
        fxConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

        /*Don't allow either motor to run backwards
     leftShoot.configNominalOutputReverse(0);
        rightShoot.configNominalOutputReverse(0); */

        /** --- SETS UP SETTINGS (Such as current limits) ON MOTORS AND SENSORS --- **/
        // Set up current limits
        currentLimitConfigurationMotor.currentLimit = 50; //50
        currentLimitConfigurationMotor.enable = true;
        currentLimitConfigurationMotor.triggerThresholdCurrent = 40;
        currentLimitConfigurationMotor.triggerThresholdTime = 3;
        // Implements these current limits on the motors
        leftShoot.configStatorCurrentLimit(currentLimitConfigurationMotor, 0);
        rightShoot.configStatorCurrentLimit(currentLimitConfigurationMotor, 0);

        // Set a closed-loop ramp rate on the motors
        leftShoot.configClosedloopRamp(0.1);
        rightShoot.configClosedloopRamp(0.1);
        // Enable voltage compensation for all control modes on the motors
        leftShoot.enableVoltageCompensation(true);
        rightShoot.enableVoltageCompensation(true);

        /** --- CONFIGURE PIDS --- **/
        // Implement variables into the PIDs
        configurePID(kP, kI, kD, kF);

        /** --- BRAKE MODES AND INVERSIONS --- **/
        // Sets up control mode.
        // Sets it to neutral mode so that the motors do not brake down to 0.
        leftShoot.setNeutralMode(NeutralMode.Coast);
        rightShoot.setNeutralMode(NeutralMode.Coast);
        /* Sets up inversions
        leftShoot.setInverted(false);
        rightShoot.setInverted(true); */
         
    }
    
    
    @Override
    public void periodic() {

        if (kep.getDouble(0) != kP) {
            kP = kep.getDouble(0);
            configurePID(kP, kI, kD, kF);
        }
        if (kei.getDouble(0) != kI) {
            kI = kei.getDouble(0);
            configurePID(kP, kI, kD, kF);
        }
        if (ked.getDouble(0) != kD) {
            kD = ked.getDouble(0);
            configurePID(kP, kI, kD, kF);
        }
        if (kef.getDouble(0) != kF) {
            kF = kef.getDouble(0);
            configurePID(kP, kI, kD, kF);
        }
        if (leftShooter.getDouble(0) != leftSpeed) {
            if (prevLeftSpeed == leftShooter.getDouble(0)) {
                leftCounter++;
            }
            if (leftCounter == 50){
                leftSpeed = leftShooter.getDouble(0);
                leftCounter = 0;
            }
            prevLeftSpeed = leftShooter.getDouble(0);

        }
        if (rightShooter.getDouble(0) != rightSpeed) {
            if (prevRightSpeed == rightShooter.getDouble(0)) {
                rightCounter++;
            }
            if (rightCounter == 50){
                rightSpeed = rightShooter.getDouble(0);
                rightCounter = 0;
            }
            prevRightSpeed = rightShooter.getDouble(0);

        }
      }

  public void configurePID(double kp, double ki, double kd, double kf){
        leftShoot.config_kP(0, kp);
        leftShoot.config_kI(0, ki);
        leftShoot.config_kD(0, kd);
        leftShoot.config_kF(0, kf);
        rightShoot.config_kP(0, kp);
        rightShoot.config_kI(0, ki);
        rightShoot.config_kD(0, kd);
        rightShoot.config_kF(0, kf);
  }

  public void setLeftShooterSpeed(double speed){
      leftShoot.set(ControlMode.PercentOutput, speed);
  }

  public void setRightShooterSpeed(double speed){
      rightShoot.set(ControlMode.PercentOutput, speed);
    
 }
 public void stopLeftShooter(){
     leftShoot.set(ControlMode.PercentOutput, 0);
 }
 
 public void stopRightShooter(){
     rightShoot.set(ControlMode.PercentOutput, 0);
 }
 public double getLeftShooterSpeed(){
     return leftShoot.getMotorOutputPercent();
 }
 public double getRightShooterSpeed(){
     return rightShoot.getMotorOutputPercent();
 }
 public double getLeftRPM() {
    // Encoder ticks per 100 ms
    double speed = leftShoot.getSelectedSensorVelocity();
    // Encoder ticks per second
    double tps = speed * 10;
    // Encoder revolutions per second
    double rps = tps / 2048;
    // Convert rps into revolutions per minute
    double rpm = rps * 60;
    return rpm;
  }
  public double getRightRPM() {
    // Encoder ticks per 100 ms
    double speed = rightShoot.getSelectedSensorVelocity();
    // Encoder ticks per second
    double tps = speed * 10;
    // Encoder revolutions per second
    double rps = tps / 2048;
    // Convert rps into revolutions per minute
    double rpm = rps * 60;
    return rpm;
  }
}
