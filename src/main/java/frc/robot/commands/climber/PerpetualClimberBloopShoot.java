package frc.robot.commands.climber;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

/**
 * This command runs the climber to a setpoint using FalconFX encoders.
 * @author Nicholas S
 */
public class PerpetualClimberBloopShoot extends CommandBase {
  private final Climber climber;
  private Supplier<String> rightBallColor;
  private Supplier<String> centerBallColor;
  private int travelTimer;

  /**
   * 
   * @param setpoint
   * @param climber
   */
  public PerpetualClimberBloopShoot(Supplier<String> rightBallColor, Supplier<String> centerBallColor, Climber climber) {
    this.climber = climber;
    this.rightBallColor = rightBallColor;
    this.centerBallColor = centerBallColor;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    travelTimer = 0;
  }
  @Override
  public void execute() {
    if (Constants.BallColor.getAllianceColor().toString() != rightBallColor.get() && rightBallColor.get() != "null" && centerBallColor.get() == "null") {
      climber.setPosition(climber.BLOOP_SHOOT);
    } else if (Constants.BallColor.getAllianceColor().toString() == rightBallColor.get() || travelTimer > 15) {
      travelTimer = 0;
      climber.setPosition(climber.TRAVEL_LOCATION);
    } else {
      travelTimer++;
    }
  }

  @Override
  public void end(boolean interupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}