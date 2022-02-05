package frc.robot.commands.pixy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.BallColor;
import frc.robot.commands.interfaces.AutoDrivableCommand;
import frc.robot.subsystems.AutoDrive.State;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.PixyCam;

public class PixyPickupCommand extends CommandBase implements AutoDrivableCommand {

  /**
   * Whatever ball color we want to pick up. Red, blue, or any.
   */
  public static enum PickupStrategy {
    RED,
    BLUE,
    ANY
  }

  private final PickupStrategy strategy;
  private final PixyCam pixyCam;
  private final AutoDrive autoDrive;
  private PIDController strafeController = new PIDController(0.05, 0.0, 0.0);
  private double strafeOutput = 0.0;
  private final double maxSpeed = 0.05;
  
  public PixyPickupCommand(PickupStrategy strategy, PixyCam pixyCam, AutoDrive autoDrive) {
    this.strategy = strategy;
    this.pixyCam = pixyCam;
    this.autoDrive = autoDrive;
    addRequirements(autoDrive);
  }

  @Override
  public void initialize() {
    autoDrive.setDelegate(this);
  }

  @Override
  public void execute() {
    // TODO: Abstract in to Ball class?
    Block targetBall = null;
    if (strategy == PickupStrategy.ANY) {
      // TODO: Prioritize our balls
      targetBall = pixyCam.getRedTarget() == null ? pixyCam.getBlueTarget() : pixyCam.getRedTarget();
    } else if (strategy == PickupStrategy.RED) {
      targetBall = pixyCam.getRedTarget();
    } else if (strategy == PickupStrategy.BLUE) {
      targetBall = pixyCam.getBlueTarget();
    }
    
    // TODO: Figure out what to do with our calculate here...
    if (targetBall == null) {
      return;
    }

    strafeOutput = strafeController.calculate(
      (double)targetBall.getX(),
      pixyCam.getFrameWidth() / 2.0
    );
    strafeOutput = MathUtil.clamp(strafeOutput, -maxSpeed, maxSpeed);

    SmartDashboard.putNumber("strafe", strafeOutput);
  }

  @Override
  public void end(boolean interrupted) {
    autoDrive.clearDelegate();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public State calculate(double forward, double strafe, boolean isFieldOriented) {
    return new AutoDrive.State(
      0.05, // TODO: production: should be negative
      -strafeOutput, //TODO: production: should be positive
      false
    );
  }
}
