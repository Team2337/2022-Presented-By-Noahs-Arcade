package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Vision;

public class SwitchPipeline extends InstantCommand{
  private final Vision vision;
  private final int pipeline;
  
  public SwitchPipeline(Vision vision, int pipeline) {
   this.vision = vision;
   this.pipeline = pipeline;
   addRequirements(vision);
  }
  @Override
  public void initialize() {
    vision.switchPipeLine(pipeline);
  }
}
