package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.Utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Climber;

/**
 * This command runs the climber using the stringpot or a joystick input
 * @author Nicholas S
 */

public class ClimberJoystickCommand extends CommandBase {
  // The subsystem the command runs on
  private final Climber climber;
  private final XboxController controller;
  private double output;
  private boolean shouldHoldCurrentPosition = true;
  private PIDController climberController = new PIDController(2, 0.0, 0.0);

  private static final double MAX_SPEED = 0.25;

  private static final double MIN_STRINGPOT_VALUE = Constants.Climber.START;
  private static final double MAX_STRINGPOT_VALUE = Constants.Climber.MID_RUNG;

  public ClimberJoystickCommand(XboxController controller, Climber climber) {
    this.climber = climber;
    this.controller = controller;
    addRequirements(climber);
    climberController.setTolerance(0.1); //TODO: Find out what this correlates to in inches on the climber
  }

  @Override
  public void initialize() {
    shouldHoldCurrentPosition = true;
    climberController.reset();
  }

  @Override
  public void execute() {
    //If the stringpot is greater or equal to minimum value AND lower or equal to the max value
    //then it is okay to operate as we are within our logical limits, else stop climber.
    if (climber.getStringPotVoltage() >= MIN_STRINGPOT_VALUE && climber.getStringPotVoltage() <= MAX_STRINGPOT_VALUE) {

      //Deadband makes sure slight inaccuracies in the controller does not make the controller move if it isn't touched
      double joystick = Utilities.deadband(controller.getRightY(), 0.15);
      SmartDashboard.putNumber("joystick after deadband", joystick);
      if (joystick == 0) {
        //First time through, set the position, so the robot will stay at this position while the controller is not touched, otherwise it would slip
        if (shouldHoldCurrentPosition) {
          climberController.setSetpoint(climber.getStringPotVoltage());
          shouldHoldCurrentPosition = false;
        }
        output = climberController.calculate(climber.getStringPotVoltage());
        //setTolerance only works with atSetpoint, so we make sure the climber stops within a set tolerance to prevent oscilation.
        if(climberController.atSetpoint()) {
          output = 0;
        }
      }
      else {
        /* The joystick is being used, use that value to move 
        the climber up and down while resetting the 'system reset' tracker */
        shouldHoldCurrentPosition = true;
        output =  joystick;

      }
      // limit speed to maxSpeed
      output =  MathUtil.clamp(output, -MAX_SPEED, +MAX_SPEED);
      climber.setSpeed(output);

    } 
    else {
      // if stringpot out of acceptable range (or zero because it's disconnected), then stop motor
      // In the future, potentially we can use the motors encoders for holding the position if the string pot gets disconnected, but that is a stretch goal right now
      climber.stop();
    }   
  }

  @Override
  public void end(boolean interupted) {
    shouldHoldCurrentPosition = true;
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}