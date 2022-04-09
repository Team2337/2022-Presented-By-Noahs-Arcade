package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * This command runs the climber to a setpoint using FalconFX encoders.
 * @author Nicholas S
 */
public class ClimberSetpointCommand extends CommandBase {
  private final Climber climber;
  private double setpoint;

  /**
   * 
   * @param setpoint
   * @param climber
   */
  public ClimberSetpointCommand(double setpoint, Climber climber) {
    this.climber = climber;
    this.setpoint = setpoint;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    // Sets a setpoint only if the <start> button has been pressed previously so that ClimberCommand has run atleast once and if we have a stringpot.
    if(climber.getStringPotHealth()){
      climber.setPosition(setpoint);
    }
  }
  @Override
  public void execute() {}

  @Override
  public void end(boolean interupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}