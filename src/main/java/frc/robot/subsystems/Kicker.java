package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotType;
import frc.robot.RobotType.Type;

public class Kicker extends SubsystemBase {

  private TalonFX motor = new TalonFX(Constants.KICKER_MOTOR);

  public Kicker() {
    motor.configFactoryDefault();
    //Invert the kicker motors on comp and practice bots because they are on different sides.
    motor.setInverted((RobotType.getRobotType() == Type.PRACTICE) ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
    motor.setNeutralMode(NeutralMode.Brake);

    motor.configOpenloopRamp(0.5);
  }

  public void setSpeed(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }

  public void start() {
    setSpeed(0.5);
  }

  public void reverse() {
    setSpeed(-0.5);
  }

  public void stop() {
    setSpeed(0.0);
  }

}
