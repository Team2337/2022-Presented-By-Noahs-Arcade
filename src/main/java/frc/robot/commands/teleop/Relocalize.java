package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class Relocalize extends InstantCommand{
  private final Vision vision;
  private final Drivetrain drivetrain;
  
  public Relocalize(Vision vision, Drivetrain drivetrain) {
   this.vision = vision;
   this.drivetrain = drivetrain;
   addRequirements(vision);
  }
  @Override
  public void initialize() {
    if (vision.activeTarget()) {
      drivetrain.resetPosition(new Pose2d(vision.getPolarResetCoordinates().toFieldCoordinate(), Rotation2d.fromDegrees(drivetrain.getGyroscopeCalculateOffset() - 90)));
    }
  }
}
