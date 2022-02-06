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
 */
public class ClimberCommand extends CommandBase {
  // The subsystem the command runs on
  private final Climber subsystem;
  private final XboxController controller;
  private double setpoint;
  private double position;
  private double position2;
  private double speed;
  private double output;
  private boolean auto;
  private boolean firstTime = true;
  private PIDController climberController = new PIDController(0.004, 0.0, 0.0);

  public ClimberCommand(Climber subsystem, XboxController controller, double setpoint, boolean auto) {
    this.subsystem = subsystem;
    this.setpoint = setpoint;
    this.controller = controller;
    this.auto = auto;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute(){
    //If the stringpot reads zero (gets disconnected), or is not autonomous, run this loop
    if (auto == false || subsystem.stringPot.getVoltage() == 0){
      //Deadband makes sure slight inaccuracies in the controller does not make the controller move if it isn't touched
      double deadband = Utilities.deadband(controller.getRightY(), 0.06);
      if (deadband == 0){
        //First time through, set the position, so the robot will stay at this position while the controller is not touched, runs by default
        if (firstTime == true){
          position = subsystem.getMotorOnePosition();
          firstTime = false;
        }
        //Holds the climber at set position 
        subsystem.holdClimber(position);
      }
      else{
        /* The controller is being pressed, use that value to move 
        the climber up and down while resetting the loop in case the controller stops being touched again */
        firstTime = true;
        subsystem.startClimber(-deadband);
      }
    }
    else{
      //Runs a PID to get the climber to the set position, as designated by the stringpot. Slows down as it reaches target. 
      output = climberController.calculate(subsystem.stringPot.getVoltage(), setpoint);
      speed =  MathUtil.clamp(output, -1, 1);
      SmartDashboard.putNumber("PID Output", output);
      SmartDashboard.putNumber("PID Speed", speed);
      subsystem.startClimber((speed * 100)); 
    }
    
}

  @Override
  public void end(boolean interupted){
    subsystem.stopClimber();
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}