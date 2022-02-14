package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utilities;

import com.ctre.phoenix.motorcontrol.MotorCommutation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Climber;

/**
 * This command runs the climber using the stringpot or a joystick input
 * @author Nicholas S
 */
public class ClimberCommand extends CommandBase {
  // The subsystem the command runs on
  private final Climber climber;
  private final XboxController controller;
  private double setpoint;
  private double position;
  private boolean useStringPot;
  private boolean shouldHoldPosition = true;
  private PIDController climberController = new PIDController(2, 0.0, 0.0);

  public ClimberCommand(XboxController controller, double setpoint, boolean useStringPot, Climber climber) {
    this.climber = climber;
    this.setpoint = setpoint;
    this.controller = controller;
    this.useStringPot = useStringPot;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climberController.setTolerance(0.1);
  }

  @Override
  public void execute(){
    //If the stringpot reads zero (gets disconnected), or is not useStringPotnomous, run this loop
    if (useStringPot == false || climber.getStringPotVoltage() == 0){
      //Deadband makes sure slight inaccuracies in the controller does not make the controller move if it isn't touched
      double deadband = Utilities.deadband(controller.getRightY(), 0.06);
      SmartDashboard.putNumber("Deadband", deadband);
      if (deadband == 0){
        //First time through, set the position, so the robot will stay at this position while the controller is not touched, otherwise it would slip
        if (shouldHoldPosition){
          position = climber.getMotorOnePosition();
          shouldHoldPosition = false;
        }
        climber.hold(position);
      }
      else{
        /* The controller is being pressed, use that value to move 
        the climber up and down while resetting the loop in case the controller stops being touched again */
        shouldHoldPosition = true;
        climber.start(-deadband);
      }
    }
    else{
      //Runs a PID to get the climber to the set position, as designated by the stringpot. Slows down as it reaches target. 
      double output = climberController.calculate(climber.getStringPotVoltage(), setpoint);
      double speed =  MathUtil.clamp(output, -1, 1);
      SmartDashboard.putNumber("PIDD Error", climberController.getPositionError());
      SmartDashboard.putNumber("PIDD Output", output);
      SmartDashboard.putNumber("PIDD Speed", speed);
      climber.start(speed); //Takes the PID output and multiplies into a number large enough to run a motor slowly.
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