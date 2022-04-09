package frc.robot.commands.kicker;

import frc.robot.Constants;
import frc.robot.subsystems.Kicker;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the intake motor
 * @author Madison J.
 *
 */
public class PerpetualKicker extends CommandBase {

  private final Kicker kicker;
  private Supplier<String> leftBallColor;

  public PerpetualKicker(Supplier<String> leftBallColor, Kicker kicker) {
    this.leftBallColor = leftBallColor;
    this.kicker = kicker;

    addRequirements(kicker);
  }

  @Override
  public void execute() {
    if (Constants.BallColor.getAllianceColor().toString() == leftBallColor.get()) {
      kicker.start(0.5);
    } else {
      kicker.start(-0.1);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}