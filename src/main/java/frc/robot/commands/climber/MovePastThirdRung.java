package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.Climber;

/**
 * This command runs the climber using the stringpot or a joystick input
 * @author Nicholas S
 */
public class MovePastThirdRung extends CommandBase {
  private final Climber climber;
  private final XboxController controller;
  
  private double setpoint;
  private boolean needsSetPoint;
  private Supplier<Rotation2d> robotPitch;

  public MovePastThirdRung(Supplier<Rotation2d> robotPitch, XboxController controller, Climber climber) {
    this.climber = climber;
    this.controller = controller;
    this.robotPitch = robotPitch;
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
        if (robotPitch.get().getDegrees() < 2.3){
          needsSetPoint = true;
          climber.setSpeed(output);
        }
        else {
          climber.stop();
        }
      }
    }

  @Override
  public void end(boolean interupted){
    
  }

  @Override
  public boolean isFinished() {
    return (!climber.climberWithinThirdRungRange());
  }
}