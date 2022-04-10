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
public class PerpetualBloopKicker extends CommandBase {

  private final Kicker kicker;
  private Supplier<String> leftBallColor;
  private Supplier<Boolean> isAtSetpoint;

  public PerpetualBloopKicker(Supplier<Boolean> isAtSetpoint, Supplier<String> leftBallColor, Kicker kicker) {
    this.leftBallColor = leftBallColor;
    this.isAtSetpoint = isAtSetpoint;
    this.kicker = kicker;

    addRequirements(kicker);
  }

  @Override
  public void execute() {
    kicker.start(0.5);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}