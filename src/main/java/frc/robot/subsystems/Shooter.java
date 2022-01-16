package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/* @nicholas.stokes If you’d like to own getting the shooter code setup
 and ready to go that would be good. Two Falcons. Make sure we have some Dashboard element pre-built using code that we can 
 configure things like the RPM for each motor, along with the PIDs (keep in mind the PIDs are going to be on the Falcon’s)
*/
public class Shooter extends SubsystemBase {

    private TalonFX leftShooterMotor;
    private TalonFX rightShooterMotor;
    public TalonFXConfiguration FXConfig;
    public StatorCurrentLimitConfiguration currentLimitConfigurationMotor = new StatorCurrentLimitConfiguration();
    private double kP = 1.15;
    private double kI = 0;
    private double kD = 0.0002;
    private double kF = 0;

    public Shooter() {
        leftShooterMotor = new TalonFX(Constants.SHOOTERLEFTMOTOR);
        rightShooterMotor = new TalonFX(Constants.SHOOTERRIGHTMOTOR);
        FXConfig = new TalonFXConfiguration();
        

        /** --- CONFIGURE MOTOR AND SENSOR SETTINGS --- **/
        // Configures motors to factory default
        leftShooterMotor.configFactoryDefault();
        rightShooterMotor.configFactoryDefault();
        // Configures sensors for PID calculations
        FXConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

        /*Don't allow either motor to run backwards
        leftShooterMotor.configNominalOutputReverse(0);
        rightShooterMotor.configNominalOutputReverse(0); */

        /** --- SETS UP SETTINGS (Such as current limits) ON MOTORS AND SENSORS --- **/
        // Set up current limits
        currentLimitConfigurationMotor.currentLimit = 50; //50
        currentLimitConfigurationMotor.enable = true;
        currentLimitConfigurationMotor.triggerThresholdCurrent = 40;
        currentLimitConfigurationMotor.triggerThresholdTime = 3;
        // Implements these current limits on the motors
        leftShooterMotor.configStatorCurrentLimit(currentLimitConfigurationMotor, 0);
        rightShooterMotor.configStatorCurrentLimit(currentLimitConfigurationMotor, 0);

        // Set a closed-loop ramp rate on the motors
        leftShooterMotor.configClosedloopRamp(0.1);
        rightShooterMotor.configClosedloopRamp(0.1);
        // Enable voltage compensation for all control modes on the motors
        leftShooterMotor.enableVoltageCompensation(true);
        rightShooterMotor.enableVoltageCompensation(true);

        /** --- CONFIGURE PIDS --- **/
        // Set variables
        
        // Implement variables into the PIDs
        configurePID(kP, kI, kD, kF);

        /** --- BRAKE MODES AND INVERSIONS --- **/
        // Sets up control mode.
        // Sets it to neutral mode so that the motors do not brake down to 0.
        leftShooterMotor.setNeutralMode(NeutralMode.Coast);
        rightShooterMotor.setNeutralMode(NeutralMode.Coast);
        /* Sets up inversions
        leftShooterMotor.setInverted(false);
        rightShooterMotor.setInverted(true); */
    }

    @Override
    public void periodic() {
        NetworkTableEntry kep = Shuffleboard.getTab("Example Tab")
        .add("kP", 0 )
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();    // This method will be called once per scheduler run

         kP = kep.getDouble(0);
         SmartDashboard.putNumber("Testing", kP);

         NetworkTableEntry kei = Shuffleboard.getTab("Example Tab")
        .add("kI", 0 )
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();    // This method will be called once per scheduler run

         kI = kei.getDouble(0);
         SmartDashboard.putNumber("Testing1", kI);

         NetworkTableEntry ked = Shuffleboard.getTab("Example Tab")
        .add("kD", 0 )
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();    // This method will be called once per scheduler run

         kD = ked.getDouble(0);
         SmartDashboard.putNumber("Testing2", kD);

         NetworkTableEntry kef = Shuffleboard.getTab("Example Tab")
        .add("kF", 0 )
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();    // This method will be called once per scheduler run

         kF = kef.getDouble(0);
         SmartDashboard.putNumber("Testing", kF);
  }

  public void configurePID(double kp, double ki, double kd, double kf){
      leftShooterMotor.config_kP(0, kp);
      leftShooterMotor.config_kI(0, ki);
      leftShooterMotor.config_kD(0, kd);
      leftShooterMotor.config_kF(0, kf);
      rightShooterMotor.config_kP(0, kp);
      rightShooterMotor.config_kI(0, ki);
      rightShooterMotor.config_kD(0, kd);
      rightShooterMotor.config_kF(0, kf);
  }


}
