package frc.robot.commands.teleop;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.commands.HeadingToTargetCommand;
import frc.robot.subsystems.Heading;

/**
 * Same command as the HeadingToTargeCommand but we will finish
 * the command once we achieve the desired heading. This is useful
 * for when we want to sequence rotating our heading to the target
 * *and then* moving.
 */
public class HeadingToTargetSequenceCommand extends HeadingToTargetCommand {

  public HeadingToTargetSequenceCommand(Supplier<Translation2d> robotTranslationSupplier, Heading heading) {
    this(Constants.kHub, robotTranslationSupplier, heading);
  }

  public HeadingToTargetSequenceCommand(Translation2d target, Supplier<Translation2d> robotTranslationSupplier, Heading heading) {
    super(target, robotTranslationSupplier, heading);
  }

  @Override
  public void initialize() {
    super.initialize();

    heading.enableMaintainHeading();
  }

  @Override
  public boolean isFinished() {
    return heading.atMaintainHeading();
  }

}
