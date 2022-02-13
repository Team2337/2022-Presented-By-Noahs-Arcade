package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import java.util.Map;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {
  private TalonFX motor = new TalonFX(Constants.KICKER_MOTOR);;

  private ShuffleboardTab tab = Shuffleboard.getTab("Kicker");
  public NetworkTableEntry kickerSpeedPercentageWidget = tab
    .add("Kicker Speed (Percentage)", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1))
    .getEntry();

  public Kicker() {
    motor.configFactoryDefault();

    motor.setInverted(TalonFXInvertType.Clockwise);
    motor.setNeutralMode(NeutralMode.Coast);

    motor.configOpenloopRamp(0.5);
  }

  public void start(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

}
