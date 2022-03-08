package frc.robot.commands.climber;

import java.util.function.Supplier;

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
  
  private final Supplier<Boolean> driverOverrideSupplier;
  private double setpoint;
  private boolean needsSetPoint;

  public ClimbSequence(double position, Supplier<Boolean> driverOverrideSupplier, XboxController controller, Climber climber, Drivetrain drivetrain) {
    this.climber = climber;
    this.controller = controller;
    this.drivetrain = drivetrain;
    this.setpoint = position;
    this.driverOverrideSupplier = driverOverrideSupplier;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    needsSetPoint = true;
    climber.activateClimber();
  }

  @Override
  public void execute(){  
    double output = -1 * Utilities.deadband(controller.getRightY(), 0.15);
    boolean isDriverOverrideEnabled = driverOverrideSupplier.get();
    //If setpoint is less than zero, we are not going to a setpoint
    if (setpoint < 0){
      if (isDriverOverrideEnabled) {
        if (output == 0){
          //First time through, set the position, so the robot will stay at this position while the controller is not touched.
          if (needsSetPoint){
            setpoint = climber.getEncoderPosition();
            needsSetPoint = false;
          }
          climber.setPosition(setpoint);
        } 
        else {
          climber.setSpeed(output);
        }
      }
      else {
        //Deadband makes sure slight inaccuracies in the controller does not make the controller move if it isn't touched
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
            //Make sure we don't go too far up for when we want to reach the mid rung, so it is easier to go down
            if (climber.getStringPotVoltage() <= climber.MID_RUNG){
              climber.setSpeed(output);
            }
            else{
            climber.stop();
            }
          }
          else {
            //If we are within the range of the third rung, we want to check our pitch to make sure we don't go under the high bar
            if ((output < 0) && climber.climberWithinThirdRungRange()) {
              if (drivetrain.getGyroscopePitch().getDegrees() < 2.3){
                climber.setSpeed(output);
              } 
              else {
                climber.stop();
              }
            }
            // If the hooks are set, don't go down any farther
            else{
              if ((output < 0) && climber.HOOKS_ARE_SET >= climber.getStringPotVoltage()) {
                climber.stop();
              }
              else {
              //Otherwise run normally downwards
              climber.setSpeed(output);
              }
            }
          }
        }
      }
    }
    else {
      //Go to setpoint if we have a valid setpoint
      if(climber.getClimberStatus()){
        climber.setPosition(setpoint);
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