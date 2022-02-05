package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utilities;

import com.ctre.phoenix.motorcontrol.MotorCommutation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Climber;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class ClimberCommand extends CommandBase {
  // The subsystem the command runs on
  private final Climber subsystem;
  private final XboxController controller;
  private double setpoint;
  private double speed;
  private double output;
  private PIDController climberController = new PIDController(0.004, 0.0, 0.0);

  public ClimberCommand(Climber subsystem, XboxController controller, double setpoint) {
    this.subsystem = subsystem;
    this.setpoint = setpoint;
    this.controller = controller;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute(){
    output = climberController.calculate(subsystem.stringPot.getVoltage(), setpoint);
    speed =  MathUtil.clamp(output, 0, 1);
    SmartDashboard.putNumber("PID Output", output);
    SmartDashboard.putNumber("PID Speed", speed);
    subsystem.startClimber((speed * 1000));
}

  @Override
  public void end(boolean interupted){
    subsystem.stopClimber();
  }
  @Override
  public boolean isFinished() {
    return (speed == 0);
  }
}