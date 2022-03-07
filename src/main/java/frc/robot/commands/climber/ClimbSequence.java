package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
/**
 * This command runs the climber sequence 
 * @author Nicholas S
 */
public class ClimbSequence extends CommandBase {
  private final Climber climber;
  private final Drivetrain drivetrain;
  private final XboxController controller;
  
  private double setpoint;
  private boolean needsSetPoint;

  public ClimbSequence(XboxController controller, Climber climber, Drivetrain drivetrain) {
    this.climber = climber;
    this.controller = controller;
    this.drivetrain = drivetrain;
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
      needsSetPoint = true;
      //If going up, make sure we don't go over max
      if (output > 0){
        //Make sure we don't go too far in clamping
        if (climber.getStringPotVoltage() <= climber.MID_RUNG){
          climber.setSpeed(output);
        }
        else{
          climber.stop();
        }
      }
      else 
        if ((output < 0) && climber.climberWithinThirdRungRange()){
          if (drivetrain.getGyroscopePitch().getDegrees() < 2.3){
            climber.setSpeed(output);
          } 
          else {
            climber.stop();
          }
        }
        else if ((output < 0) && climber.HOOKS_ARE_SET >= climber.getStringPotVoltage()){
          climber.stop();
        }
        else {
          climber.setSpeed(output);
        }
      }
    }

  @Override
  public void end(boolean interupted){
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}