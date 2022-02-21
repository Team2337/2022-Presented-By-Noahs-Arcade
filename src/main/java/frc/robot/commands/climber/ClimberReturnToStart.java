package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * This command runs the climber back to the START position using the FalconFX encoders,
 * regardless if the <start> button has been pressed to activate the climber subsystem.
 * @author Nicholas S
 */
public class ClimberReturnToStart extends CommandBase {
  private final Climber climber;

  public ClimberReturnToStart(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setPositionUsingEncoder(climber.START);
  }
  @Override
  public void execute(){
    }
  @Override
  public void end(boolean interupted){
    climber.stop();
  }
  @Override
  public boolean isFinished() {
    return true;
  }
}