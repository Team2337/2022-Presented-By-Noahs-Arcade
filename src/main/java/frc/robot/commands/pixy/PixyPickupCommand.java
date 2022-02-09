package frc.robot.commands.pixy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

  private double forwardOutput = 0.0;
  private double strafeOutput = 0.0;

  private final double maxForwardSpeed = 0.05;
  private final double maxStrafeSpeed = 0.05;

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

    forwardOutput = 0.0;
    strafeOutput = 0.0;

    // TODO: Figure out what to do with our calculate here...
    if (targetBall == null) {
      return;
    }

    // Negative since our Pixy cam is on the back of our robot. In order to move
    // TOWARDS the ball, we need to move backwards.
    forwardOutput = -maxForwardSpeed;

    strafeOutput = strafeController.calculate(
      (double)targetBall.getX(),
      pixyCam.getFrameWidth() / 2.0
    );

    // Negative since our Pixy cam is on the back of our robot. Our
    // side-to-side values need to be inverted, since our side-to-side
    // values are relative to the front of the robot
    strafeOutput = -strafeOutput;
    strafeOutput = MathUtil.clamp(strafeOutput, -maxStrafeSpeed, maxStrafeSpeed);

    SmartDashboard.putNumber("strafe", strafeOutput);
  }

  @Override
  public void end(boolean interrupted) {
    autoDrive.clearDelegate();
  }

  @Override
  public boolean isFinished() {
    // TODO: when should this end?
    return false;
  }

  @Override
  public State calculate(double forward, double strafe, boolean isFieldOriented) {
    // TODO: what happens when we DON'T see a ball?
    return new AutoDrive.State(
      forwardOutput,
      strafeOutput,
      false
    );
  }
}
