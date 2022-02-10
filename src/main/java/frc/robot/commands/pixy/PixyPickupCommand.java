package frc.robot.commands.pixy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
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
  private boolean seesBall = false;

  private final double maxForwardSpeed = 0.2;
  private final double maxStrafeSpeed = 0.2;

  public PixyPickupCommand(PickupStrategy strategy, PixyCam pixyCam, AutoDrive autoDrive) {
    this.strategy = strategy;
    this.pixyCam = pixyCam;
    this.autoDrive = autoDrive;

    addRequirements(autoDrive);
  }

  @Override
  public void initialize() {
    seesBall = false;
    autoDrive.setDelegate(this);
  }

  @Override
  public void execute() {
    // TODO: Abstract in to Ball class?
    Block targetBall = null;
    if (strategy == PickupStrategy.ANY) {
      switch (DriverStation.getAlliance()) {
        default:
        case Red:
          // If alliance is red, prioritize red targets if they are there; otherwise get blue targets
          targetBall = pixyCam.getRedTarget() == null ? pixyCam.getBlueTarget() : pixyCam.getRedTarget();
          break;
        case Blue:
          // If alliance is blue, prioritize blue targets if they are there; otherwise get red targets
          targetBall = pixyCam.getBlueTarget() == null ? pixyCam.getRedTarget() : pixyCam.getBlueTarget();
          break;
      }
    } else if (strategy == PickupStrategy.RED) {
      targetBall = pixyCam.getRedTarget();
    } else if (strategy == PickupStrategy.BLUE) {
      targetBall = pixyCam.getBlueTarget();
    }

    forwardOutput = 0.0;
    strafeOutput = 0.0;

    seesBall = targetBall != null;
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
    /**
     * Here's an idea about how to do this:
     * When the intake code gets finished, this will be tied in with the intake command.
     * Currently, that intake command is bundled with the delivery code, it might need
     * to be separated into its own branch.
     */
    return false;
  }

  @Override
  public State calculate(double forward, double strafe, boolean isFieldOriented) {
    if (seesBall) {
      return new AutoDrive.State(
        forwardOutput,
        strafeOutput,
        false
      );
    } else {
      return null;
    }
  }
}
