package frc.robot.commands.pixy;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.interfaces.AutoDrivableCommand;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.AutoDrive.State;
import frc.robot.subsystems.hardware.PixyCam;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import frc.robot.subsystems.AutoDrive;

public class PixyPickupCommand extends CommandBase implements AutoDrivableCommand {

  /**
   * Whatever ball color we want to pick up. Red, blue, or any.
   */
  public static enum PickupStrategy {
    RED,
    BLUE,
    ANY
  }

  // The position in the frame where the ball will go max speed.
  // Ex: 1 == at the far edge of the frame, the ball will go max speed.
  // If the ball is further in the frame, we will scale our speed.
  // Ex: 2 == until the ball is halfway through the half frame (so 1/4
  // through the full frame) we will go full speed towards it, then
  // start scaling our speed.
  // This is useful for the buffer zones on the side of the frame where
  // we don't detect a ball right away but don't want to go slower until
  // we absolutely have to.
  private static final double MAX_SPEED_HALF_FRAME_SCALE = 1.1;
  private static final double MAX_STRAFE_OUTPUT = 0.5;
  private static final double LAST_SEEN_CYCLE_COUNTER_MAX = 50; // 1s

  private final AutoDrive autoDrive;
  private final PixyCam pixyCam;

  private final PIDController strafeController = new PIDController(0.0035, 0.0, 0.0);

  private PickupStrategy strategy;

  private Block targetBall;
  private int lastSeenCycleCounter = 0;
  private double strafeOutput = 0.0;

  public PixyPickupCommand(AutoDrive autoDrive, PixyCam pixyCam) {
    this.pixyCam = pixyCam;
    this.autoDrive = autoDrive;

    addRequirements(autoDrive);
  }

  @Override
  public void initialize() {
    autoDrive.setDelegate(this);

    resetInternalState();
  }

  private void log() {
    String strategyString = "N/A";
    if (strategy != null) {
      strategyString = strategy.toString();
    }
    Logger.getInstance().recordOutput("PixyPickup/Strategy", strategyString);
    Logger.getInstance().recordOutput("PixyPickup/Last Seen Counter", lastSeenCycleCounter);
    Logger.getInstance().recordOutput("PixyPickup/Strafe Output", strafeOutput);
    Logger.getInstance().recordOutput("PixyPickup/Controller Error", strafeController.getPositionError());
  }

  public void setStrategy(PickupStrategy strategy) {
    if (this.strategy != strategy) {
      resetInternalState();
    }
    this.strategy = strategy;
  }

  public void clearStrategy() {
    setStrategy(null);
  }

  private void resetInternalState() {
    targetBall = null;
    lastSeenCycleCounter = 0;
    strafeOutput = 0.0;

    strafeController.reset();
  }

  @Override
  public void execute() {
    log();

    if (strategy == null) {
      return;
    }

    Block latestTargetBall = null;
    if (strategy == PickupStrategy.ANY) {
      switch (DriverStation.getAlliance()) {
        default:
        case Red:
          // If alliance is red, prioritize red targets if they are there;
          // otherwise get blue targets
          latestTargetBall = pixyCam.getRedTarget() == null ? pixyCam.getBlueTarget() : pixyCam.getRedTarget();
          break;
        case Blue:
          // If alliance is blue, prioritize blue targets if they are there;
          // otherwise get red targets
          latestTargetBall = pixyCam.getBlueTarget() == null ? pixyCam.getRedTarget() : pixyCam.getBlueTarget();
          break;
      }
    } else if (strategy == PickupStrategy.RED) {
      latestTargetBall = pixyCam.getRedTarget();
    } else if (strategy == PickupStrategy.BLUE) {
      latestTargetBall = pixyCam.getBlueTarget();
    }

    // If our `latestTargetBall` is null, remember where our last seen ball was
    // until we haven't seen a new ball ball for LAST_SEEN_CYCLE_COUNTER_MAX
    // number of cycles.
    if (targetBall != null && latestTargetBall == null) {
      lastSeenCycleCounter++;
    } else {
      lastSeenCycleCounter = 0;
      targetBall = latestTargetBall;
    }
    if (lastSeenCycleCounter >= LAST_SEEN_CYCLE_COUNTER_MAX) {
      targetBall = null;
    }

    strafeOutput = 0.0;

    if (targetBall == null) {
      return;
    }

    double frameCenter = pixyCam.getFrameWidth() / 2.0;
    double output = strafeController.calculate(
      (double) targetBall.getX(),
      frameCenter
    );

    // Determine our maximum output based on the half-frame size + our P value
    // and scale our output so we'll move full speed until we hit our
    // MAX_SPEED_HALF_FRAME_SCALE position in the half frame.
    double scaledOutput = output / (((frameCenter / MAX_SPEED_HALF_FRAME_SCALE) * strafeController.getP()));
    // Square our output so we more quickly ramp to zero as we approach the center of our frame.
    strafeOutput = Utilities.squareValues(scaledOutput) * MAX_STRAFE_OUTPUT;

    // Negative since our Pixy cam is on the back of our robot. Our
    // side-to-side values need to be inverted, since our side-to-side
    // values are relative to the front of the robot
    strafeOutput = MathUtil.clamp(
      -strafeOutput,
      -MAX_STRAFE_OUTPUT,
      MAX_STRAFE_OUTPUT
    );
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
    if (targetBall != null) {
      return new AutoDrive.State(
        forward,
        strafeOutput
      );
    } else {
      return null;
    }
  }

}
