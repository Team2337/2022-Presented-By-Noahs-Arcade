package frc.robot.subsystems;


import java.util.Map;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/* @nicholas.stokes If you’d like to own getting the shooter code setup
 and ready to go that would be good. Two Falcons. Make sure we have some Dashboard element pre-built using code that we can 
 configure things like the RPM for each motor, along with the PIDs (keep in mind the PIDs are going to be on the Falcon’s)
*/
public class Shooter extends SubsystemBase {

    public TalonFX topShoot;
    public TalonFX bottomShoot;
    public TalonFX kicker;
    public TalonFXConfiguration fxConfig;
    public StatorCurrentLimitConfiguration currentLimitConfigurationMotor = new StatorCurrentLimitConfiguration();

    private double kP = 1.15;
    private double kI = 0;
    private double kD = 0.0002;
    private double kF = 0.1079;

    private double topSpeed = 0;
    private double bottomSpeed = 0;
    private double prevTopSpeed = 0;
    private double prevBottomSpeed = 0;
    private int topCounter = 0;
    private int bottomCounter = 0;
    private double motorShutdownTemp = 70;
    public boolean motorOverTemp = false;

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

     public NetworkTableEntry topShooter = speeds
        .add("Top Shooter Speed", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

        ShuffleboardLayout temps = tab.getLayout("Shooter Temperature", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(4, 0); 

        ShuffleboardLayout speed = tab.getLayout("Motor Velocities", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(6, 0); 
        
        
        /*
     public NetworkTableEntry topShootTemp = temps
     .add("Top Shooter Temp", topShoot.getTemperature())
     .withWidget(BuiltInWidgets.kDial)
     .getEntry();
     public NetworkTableEntry bottomShootTemp = temps
     .add("Bottom Shooter Temp", bottomShoot.getTemperature())
     .withWidget(BuiltInWidgets.kDial)
     .getEntry();
     public NetworkTableEntry OverTemp = temps
     .add("OverTemp", motorOverTemp)
     .withWidget(BuiltInWidgets.kBooleanBox)
     .getEntry();
     */

    public Shooter() {
        topShoot = new TalonFX(1);
        //topShoot = new TalonFX(Constants.SHOOTER_LEFT_MOTOR);
        bottomShoot = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR);
        //kicker = new TalonFX(Constants.KICKER_MOTOR);
        fxConfig = new TalonFXConfiguration();
        

        /** --- CONFIGURE MOTOR AND SENSOR SETTINGS --- **/
        // Configures motors to factory default
        topShoot.configFactoryDefault();
        bottomShoot.configFactoryDefault();
        // Configures sensors for PID calculations
        fxConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

        /*Don't allow either motor to run backwards
        topShoot.configNominalOutputReverse(0);
        bottomShoot.configNominalOutputReverse(0); */

        /** --- SETS UP SETTINGS (Such as current limits) ON MOTORS AND SENSORS --- **/
        // Set up current limits
        currentLimitConfigurationMotor.currentLimit = 50; //50
        currentLimitConfigurationMotor.enable = true;
        currentLimitConfigurationMotor.triggerThresholdCurrent = 40;
        currentLimitConfigurationMotor.triggerThresholdTime = 3;
        // Implements these current limits on the motors
        topShoot.configStatorCurrentLimit(currentLimitConfigurationMotor, 0);
        bottomShoot.configStatorCurrentLimit(currentLimitConfigurationMotor, 0);

        // Set a closed-loop ramp rate on the motors
        topShoot.configClosedloopRamp(0.1);
        bottomShoot.configClosedloopRamp(0.1);
        // Enable voltage compensation for all control modes on the motors
        topShoot.enableVoltageCompensation(true);
        bottomShoot.enableVoltageCompensation(true);

        /** --- CONFIGURE PIDS --- **/
        // Implement variables into the PIDs
        configurePID(kP, kI, kD, kF);

        /** --- BRAKE MODES AND INVERSIONS --- **/
        // Sets up control mode.
        // Sets it to neutral mode so that the motors do not brake down to 0.
        topShoot.setNeutralMode(NeutralMode.Coast);
        bottomShoot.setNeutralMode(NeutralMode.Coast);
        // Sets up inversions
        topShoot.setInverted(false);
        bottomShoot.setInverted(true); 

        temps.addNumber("Top Shooter Temperature", () -> topShoot.getTemperature());
        temps.addNumber("Bottom Shooter Temperature", () -> bottomShoot.getTemperature());
        temps.addBoolean("Motors Overheating?", () -> motorOverTemp);
        speed.addNumber("Top Shooter RPM", () -> getTopRPM());
        speed.addNumber("Bottom Shooter RPM", () -> getBottomRPM());
        speed.addNumber("Top Shooter Wheel Speed", () -> getTopWheelSpeed());
        speed.addNumber("Bottom Shooter Wheel Speed", () -> getBottomWheelSpeed());

        
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
        if (topShooter.getDouble(0) != topSpeed) {
            if (prevTopSpeed == topShooter.getDouble(0)) {
                topCounter++;
            }
            if (topCounter == 50){
                topSpeed = topShooter.getDouble(0);
                topCounter = 0;
            }
            prevTopSpeed = topShooter.getDouble(0);

        }

        if (getBottomMotorOverTemp() | getTopMotorOverTemp()) {
            motorOverTemp = true;
        } else {
            motorOverTemp = false;
        }
      }

  public void configurePID(double kp, double ki, double kd, double kf){
        topShoot.config_kP(0, kp);
        topShoot.config_kI(0, ki);
        topShoot.config_kD(0, kd);
        topShoot.config_kF(0, kf);
        bottomShoot.config_kP(0, kp);
        bottomShoot.config_kI(0, ki);
        bottomShoot.config_kD(0, kd);
        bottomShoot.config_kF(0, kf);
  }


 public void stopTopShooter(){
     topShoot.set(ControlMode.PercentOutput, 0);
 }
 
 public void stopBottomShooter(){
     bottomShoot.set(ControlMode.PercentOutput, 0);
 }

 /*public void startKicker(){
     kicker.set(ControlMode.PercentOutput, 100);
 }

 public void stopKicker(){
     kicker.set(ControlMode.PercentOutput, 0); */
 
 public double getTopShooterSpeed(){
     return topShoot.getMotorOutputPercent();
 }
 public double getBottomShooterSpeed(){
     return bottomShoot.getMotorOutputPercent();
 }

 public boolean getTopMotorOverTemp() {
     return topShoot.getTemperature() > motorShutdownTemp;
 }

 public boolean getBottomMotorOverTemp() {
    return bottomShoot.getTemperature() > motorShutdownTemp;
}

 public double getTopRPM() {
    // Encoder ticks per 100 ms
    double speed = topShoot.getSelectedSensorVelocity();
    // Encoder ticks per second
    double tps = speed * 10;
    // Encoder revolutions per second
    double rps = tps / 2048;
    // Convert rps into revolutions per minute
    double rpm = rps * 60;
    return rpm;
  }
  public double getBottomRPM() {
    // Encoder ticks per 100 ms
    double speed = bottomShoot.getSelectedSensorVelocity();
    // Encoder ticks per second
    double tps = speed * 10;
    // Encoder revolutions per second                      
    double rps = tps / 2048;
    // Convert rps into revolutions per minute
    double rpm = rps * 60;
    return rpm;
}

  public double getTopWheelSpeed(){
      double wheelDiameter = 4; //This is in inches.
      double rpm = getTopRPM();
      double wheelRpm = rpm * (16/24); //16/24 is the gear ratio (16 is the input gear of the falcons, 24 is the output gear of the wheel)
      double wheelSpeed = ((2*Math.PI*wheelRpm)/60)*((wheelDiameter/12)/2); //This turns wheel RPM's into ft/s
      return wheelSpeed;
    }
  public double getBottomWheelSpeed(){
      double wheelDiameter = 4; //This is in inches.
      double rpm = getBottomRPM();
      double wheelRpm = rpm * (16/24); //16/24 is the gear ratio (16 is the input gear of the falcons, 24 is the output gear of the wheel)
      double wheelSpeed = ((2*Math.PI*wheelRpm)/60)*((wheelDiameter/12)/2); //This turns wheel RPM's into ft/s
      return wheelSpeed;
    }
    public void setTopShooterSpeed(double speed){
        /*double wheel = speed /((4/12)/2); //Speed is inputed in ft/s and is converted to encoder ticks per 100 ms which can be set in velocity.
        double wheelRpm = (wheel *60)/(Math.PI * 2);
        double rpm = wheelRpm / (16/24); //Gear Ratio
        double rps = rpm/60;
        double tps = rps*2048;
        double encoderTicks = tps / 10;
        topShoot.set(ControlMode.Velocity, (encoderTicks)); */
        //x RPM of a Falcon 500 is 6380 RPM, so that would be at 100% power
        double rps = 6380/60; // Max revolutions per second
        double tps = rps*2048; // Max encoder ticks per second
        double maxSpeed = tps/10; // This converts to motor ticks. 
        double speedAtOnePercent = maxSpeed/100; //Encoder ticks at 1% power?
        topShoot.set(ControlMode.Velocity, (speedAtOnePercent * speed)); 
        
     }
     
     public void setBottomShooterSpeed(double speed){
        /*double wheel = speed /((4/12)/2); //Speed is inputed in ft/s and is converted to encoder ticks per 100 ms which can be set in velocity.
        double wheelRpm = (wheel *60)/(Math.PI * 2);
        double rpm = wheelRpm / (16/24); //Gear Ratio
        double rps = rpm/60;
        double tps = rps*2048;
        double encoderTicks = tps / 10;
        bottomShoot.set(ControlMode.Velocity, (encoderTicks)); */
        //Max RPM of a Falcon 500 is 6380 RPM, so that would be at 100% power
        double rps = 6380/60; // Max revolutions per second
        double tps = rps*2048; // Max encoder ticks per second
        double maxSpeed = tps/10; // This converts to motor ticks. 
        double speedAtOnePercent = maxSpeed/100; //Encoder ticks at 1% power?
        bottomShoot.set(ControlMode.Velocity, (speedAtOnePercent * speed)); 
        
     }
}
