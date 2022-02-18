package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerdyfiles.utilities.Utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Climber;

/**
 * This command runs the climber using the stringpot or a joystick input
 * @author Nicholas S
 */
public class ClimberDefaultCommand extends CommandBase {
  // The subsystem the command runs on
  private final Climber climber;
  private final XboxController controller;
  private double setpoint;
  private double output;
  private boolean firstTimeThru = true;
  private PIDController climberController = new PIDController(2, 0.0, 0.0);

  private double maxSpeed = 0.25;

  private double minStringpotValue = 1.0;
  private double maxStringpotValue = 2.0;

  public ClimberDefaultCommand(XboxController controller, Climber climber) {
    this.climber = climber;
    this.controller = controller;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climberController.setTolerance(0.1);
  }

  @Override
  public void execute(){

SmartDashboard.putBoolean("start button pressed", controller.getStartButton());
SmartDashboard.putNumber("joystick", controller.getRightY());
SmartDashboard.putNumber("pot value", climber.getStringPotVoltage());
SmartDashboard.putNumber("pot set point", setpoint);
SmartDashboard.putNumber("pot error", climberController.getPositionError());


    //When the start button is released, reset firstTimeThrough. This makes it so that we aren't going to a previous setpoint every
    //time we press the button and will set a new one every time.
    if(!controller.getStartButton()){
      firstTimeThru = true;
    }
    //If the stringpot is greater than minimum value AND lower than max value AND <Start> button is pressed 
    //then it is okay to operate, else stop climber.
    if (climber.getStringPotVoltage() > minStringpotValue && climber.getStringPotVoltage() < maxStringpotValue && controller.getStartButton()){


      //Deadband makes sure slight inaccuracies in the controller does not make the controller move if it isn't touched
      double joystick = Utilities.deadband(controller.getRightY(), 0.15);
      SmartDashboard.putNumber("joystick after deadband", joystick);
      if (joystick == 0){
        //First time through, set the position, so the robot will stay at this position while the controller is not touched, otherwise it would slip
        if (firstTimeThru){
          setpoint = climber.getStringPotVoltage();
          firstTimeThru = false;
        }
        output = climberController.calculate(climber.getStringPotVoltage(), setpoint);
        //setTolerance only works with atSetpoint, so we make sure the climber stops within a set tolerance to prevent oscilation.
        if(climberController.atSetpoint()){
          output = 0;
        }
      }
      else{
        /* The joystick is being used, use that value to move 
        the climber up and down while resetting the 'first time through' tracker */
        firstTimeThru = true;
        output =  joystick;

      }
      // limit speed to maxSpeed
      output =  MathUtil.clamp(output, -maxSpeed, +maxSpeed);

      climber.start(output);

    } 
    else{
      // if stringpot out of acceptable range (or zero because it's disconnected), then stop motor
      climber.stop();
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