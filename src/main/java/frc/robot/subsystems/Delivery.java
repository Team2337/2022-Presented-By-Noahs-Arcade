package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Direction;

/**
 * Subsystem for the delivery mechanism
 * 
 * @author Michael F, Nicholas S, Alex C
 */
public class Delivery extends SubsystemBase {

  // Motor
  private final TalonFX motor;

  private final XboxController controller;

  /**
   * Initializes the Delivery subsystem with its two color sensors
   */
  public Delivery(XboxController controller) {
    // Initialize motor
    motor = new TalonFX(Constants.DELIVERY_MOTOR_ID);

    // TODO: make sure config settings are correct
    //Set settings on motor
    motor.configFactoryDefault();

    motor.setInverted(false); //TODO: see what actual inversion should be
    motor.setNeutralMode(NeutralMode.Brake);

    motor.configOpenloopRamp(0.5);

    this.controller = controller;

    // Set up shuffleboard stuff
    // ShuffleboardTab deliveryTab = Shuffleboard.getTab("Delivery");

  }

  @Override
  public void periodic() {
    if (RobotState.isEnabled() && RobotContainer.operatorStation.isBlueSwitchOn()) {
      startDelivery(Direction.CLOCKWISE, Math.abs(controller.getLeftX()) > 0.1 ? controller.getLeftX() : 0);
    }
    SmartDashboard.putBoolean("On", RobotContainer.operatorStation.isBlueSwitchOn());
    SmartDashboard.putNumber("Value", controller.getLeftX());
  }


  public void startDelivery(Direction direction) {
    startDelivery(direction, Constants.DELIVERY_SPEED);
  }

  public void startDelivery(Direction direction, double speed) {
    if (direction == Direction.CLOCKWISE) {
      // Rotate motor forward
      motor.set(ControlMode.PercentOutput, speed);
    } else if (direction == Direction.COUNTER_CLOCKWISE) {
      // Rotate motor CCW
      motor.set(ControlMode.PercentOutput, -speed);
    }
  }

  /**
   * Stops the delivery mechanism
   */
  public void stopDelivery() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }

}
