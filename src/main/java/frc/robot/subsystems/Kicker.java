package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import java.util.Map;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {
 private TalonFX kicker; 
 private ShuffleboardTab tab = Shuffleboard.getTab("Kicker");
 public NetworkTableEntry kick3r = tab
        .add("Kicker Speed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();

public Kicker(){

  kicker = new TalonFX(Constants.KICKER_MOTOR);

    
  //Set settings on kicker
  kicker.configFactoryDefault();

  kicker.setInverted(false); //TODO: make sure this is correct
  kicker.setNeutralMode(NeutralMode.Coast);

  kicker.configOpenloopRamp(0.5);


}

public void startKicker(double speed){
  kicker.set(ControlMode.PercentOutput, speed);
}

public void stopKicker(){
  kicker.set(ControlMode.PercentOutput, 0); 
}

}
