package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;

public class ReverseStopShooterCommand extends StartStopShooterCommand {

  public ReverseStopShooterCommand(Shooter shooter) {
    super(-10, shooter);
  }

}
