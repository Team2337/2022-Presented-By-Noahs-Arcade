package frc.robot.commands.shooter;

import frc.robot.subsystems.Kicker;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the kicker motor
 * @author Madison J.
 *
 */
public class AutoKickerCommand extends CommandBase {

  private final Kicker kicker;
  private double wait;
  private double waitCounter;

  public AutoKickerCommand(Kicker kicker, double wait) {
    this.kicker = kicker;
    this.wait = wait;
    //Puts wait into iterations from seconds
    wait = wait / 50;
    addRequirements(kicker);
  }

  @Override
  public void initialize() {
    waitCounter = 0;
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Wait Counter", waitCounter);
    if (waitCounter >= wait) {
      kicker.start();
    } else {
      waitCounter++;
    }
  }

  @Override
  public void end(boolean interrupted) {
    kicker.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}