package frc.robot.commands.kicker;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Kicker;

/**
 * Runs the kicker motor forward after a delay.
 * @author Madison J.
 *
 */
public class ForwardKickerWithDistance extends CommandBase {
  private int isFinishedCounter;
  private boolean finished = false;
  private final Kicker kicker;
  private PolarCoordinate robotCoordinate;
  private Supplier<Translation2d> translationSupplier; 

  /**
   * Runs the kicker motor forward.
   */
  public ForwardKickerWithDistance(Supplier<Translation2d> translationSupplier, Kicker kicker) {
    this.kicker = kicker;
    this.translationSupplier = translationSupplier;

    robotCoordinate = PolarCoordinate.fromFieldCoordinate(translationSupplier.get());

    addRequirements(kicker);
  }

  @Override
  public void initialize() {
    isFinishedCounter = 0;
  }

  @Override
  public void execute() {
    robotCoordinate = PolarCoordinate.fromFieldCoordinate(translationSupplier.get());
    double distance = Units.metersToInches(robotCoordinate.getRadiusMeters());
    SmartDashboard.putNumber("Distance", distance);

    if (distance < 45 || (distance > 120 && distance < 160)) {
      kicker.start(0.5);
      isFinishedCounter++;
    }
    if (isFinishedCounter > 100) {
      finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    kicker.stop();
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

}