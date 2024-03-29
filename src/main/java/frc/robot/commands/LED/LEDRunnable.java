package frc.robot.commands.LED;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hardware.LED;

public class LEDRunnable extends CommandBase{
  private final RobotContainer robotContainer;
  private final LED led;

  public LEDRunnable(LED led, RobotContainer robotContainer) {
    this.led = led;
    this.robotContainer = robotContainer;

    addRequirements(led);
  }
  @Override
  public void execute() {
    if (robotContainer.getYellowSwitchStatus() && robotContainer.getGyroscopeRoll() < Constants.CLIMBER_ROLL) {
      led.setColor(Color.kPurple);
    } else if (robotContainer.isShooterUpToLEDSpeed() && robotContainer.hasActiveTarget()) {
      led.setColor(Color.kRed, robotContainer.getTx());
    } else if (robotContainer.isShooterUpToLEDSpeed()) {
      led.setColor(Color.kBlue);
    } else if (robotContainer.getOperatorStartStatus() || robotContainer.getOperatorBackStatus()) {
      led.setColorMiddle();
    } else if (robotContainer.hasActiveTarget()) {
      led.setColor(Color.kYellow, robotContainer.getTx());
    } else if (robotContainer.getOperatorRightTriggerStatus() && DriverStation.isTeleop()) {
      led.setColorMiddle();
    } else {
      led.setOff();
    }
  }
}