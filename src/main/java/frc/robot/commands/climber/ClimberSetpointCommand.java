package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberSetpointCommand extends CommandBase {

  private static final double MAX_SPEED = 0.10;

  private final Climber climber;
  private final XboxController controller;

  private boolean shouldHoldPositionWhenStopped = true;

  public ClimberSetpointCommand(XboxController controller, Climber climber) {
    this.climber = climber;
    this.controller = controller;

    addRequirements(climber);
  }

}
