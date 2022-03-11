package frc.robot.commands.kicker;

import frc.robot.subsystems.Kicker;
import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class WaitableKickerCommand extends CommandBase {

  private final Runnable kickerRunnable;
  private final double waitCycles;
  private final Kicker kicker;

  private int waitCounter;

  /**
   * Run a Kicker command after a set number of seconds.
   */
  public WaitableKickerCommand(Runnable kickerRunnable, double waitTimeSeconds, Kicker kicker) {
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

}
