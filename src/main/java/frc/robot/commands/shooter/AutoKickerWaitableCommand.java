package frc.robot.commands.shooter;

import frc.robot.subsystems.Kicker;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoKickerWaitableCommand extends CommandBase {

  private final Runnable kickerRunnable;
  private final double waitCycles;
  private final Kicker kicker;

  private int waitCounter;

  /**
   * Run a Kicker command after a set number of seconds.
   */
  public AutoKickerWaitableCommand(Runnable kickerRunnable, double waitTimeSeconds, Kicker kicker) {
    this.kickerRunnable = kickerRunnable;
    // Puts wait into iterations from seconds
    this.waitCycles = waitTimeSeconds / 50; // 50 == 0.02s - our cycle time
    this.kicker = kicker;

    addRequirements(kicker);
  }

  @Override
  public void initialize() {
    waitCounter = 0;
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Wait Counter", waitCounter);
    if (waitCounter >= waitCycles) {
      kickerRunnable.run();
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
