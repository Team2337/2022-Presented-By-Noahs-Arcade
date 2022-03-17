package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.Climber;

/**
 * This command runs the climber using the stringpot or a joystick input
 * @author Nicholas S
 */
public class MoveToHooksSet extends CommandBase {
  private final Climber climber;
  private final XboxController controller;
  
  private double setpoint;
  private boolean needsSetPoint;

  public MoveToHooksSet(XboxController controller, Climber climber) {
    this.climber = climber;
    this.controller = controller;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    needsSetPoint = true;
    climber.activateClimber();
  }

  @Override
  public void execute(){  
    //Deadband makes sure slight inaccuracies in the controller does not make the controller move if it isn't touched
    double output = -1 * Utilities.deadband(controller.getRightY(), 0.15);
    if (output == 0){
        //First time through, set the position, so the robot will stay at this position while the controller is not touched.
        if (needsSetPoint){
          setpoint = climber.getEncoderPosition();
          needsSetPoint = false;
        }
        climber.setPosition(setpoint);
    }
    else{
        // The joystick is being used, use that value to move the climber up and down
        // while resetting the 'first time through' tracker
        needsSetPoint = true;
        climber.setSpeed(output);
      }
    }

  @Override
  public void end(boolean interupted){
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return (climber.stringpotToEncoder(climber.HOOKS_ARE_SET) >= climber.getEncoderPosition());
  }
}