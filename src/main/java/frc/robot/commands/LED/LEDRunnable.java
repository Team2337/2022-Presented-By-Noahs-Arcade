package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
    if (robotContainer.isShooterUpToLEDSpeed() && robotContainer.isOnTarget()) {
      led.setRed();
    } else if (robotContainer.isShooterUpToLEDSpeed()) {
      led.setBlue();
    } else if (robotContainer.isOnTarget() && robotContainer.hasActiveTarget()) {
      led.setYellow();
    } else {
      led.setOff();
    }
  }
}
