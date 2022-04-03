package frc.robot.commands.pixy;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.interfaces.AutoDrivableCommand;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.AutoDrive.State;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import frc.robot.subsystems.AutoDrive;

public class PixyPickupCommand extends CommandBase implements AutoDrivableCommand {

  /**
   * Whatever ball color we want to pick up. Red, blue, or any.
   */
  public static enum PickupStrategy {
    RED,
    BLUE,
    ANY,
    OURS,
    THEIRS
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

  private final PickupStrategy strategy;
  private final AutoDrive autoDrive;
  
  private final PIDController strafeController = new PIDController(0.0035, 0.0, 0.0);
  private final Pixy2 pixycam = Pixy2.createInstance(Pixy2.LinkType.SPI);

  private Block targetBall;
  private int lastSeenCycleCounter = 0;
  private double strafeOutput = 0.0;
  private XboxController driverController;

  public Supplier<Rotation2d> gyroSupplier;

  public double spangle = 0;

  private int state;
  private int chipselect = 0;

  private PolarCoordinate joystickCoordinates = new PolarCoordinate(0, Rotation2d.fromDegrees(0));

  private enum BlockSignature {
    RED(1),
    BLUE(2);

    int value;

    BlockSignature(int value) {
      this.value = value;
    }
  }

  public PixyPickupCommand(PickupStrategy strategy, Supplier<Rotation2d> gyroSupplier, XboxController driverController, AutoDrive autoDrive) {
    this.strategy = strategy;
    this.gyroSupplier = gyroSupplier;
    this.driverController = driverController;
    this.autoDrive = autoDrive;
    
    addRequirements(autoDrive);
  }

  @Override
  public void initialize() {
    autoDrive.setDelegate(this);


    state = pixycam.init(chipselect);
    resetInternalState();
  }

  private void log() {
    String strategyString = "N/A";
    if (strategy != null) {
      strategyString = strategy.toString();
    }
    SmartDashboard.putString("PixyPickup/Strategy", strategyString);
    SmartDashboard.putNumber("PixyPickup/Last Seen Counter", lastSeenCycleCounter);
    SmartDashboard.putNumber("PixyPickup/Strafe Output", strafeOutput);
    SmartDashboard.putNumber("PixyPickup/Controller Error", strafeController.getPositionError());
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
    double y = driverController.getLeftX();
    double x = -driverController.getLeftY();
    Rotation2d joystickAngle = joystickCoordinates.fromFieldCoordinate(new Translation2d(x, y), new Translation2d(0, 0)).getTheta();
    double jAngle = -joystickAngle.getDegrees();
    double gAngle = gyroSupplier.get().getDegrees();
    double dangle = jAngle - gAngle;

    spangle = Math.cos(Math.toRadians(dangle));
    spangle = spangle * 0.4;
    
    if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1) {
      spangle = 0;
    }

    if (!isConnected()) {
      return;
    }

    if (strategy == null) {
      return;
    }

    ArrayList<Block> blocks = getBlocks();
    
    Block latestTargetBall = null;
    if (strategy == PickupStrategy.RED) {
      latestTargetBall = getLargestRedBlock(blocks);
    } else if (strategy == PickupStrategy.BLUE) {
      latestTargetBall = getLargestBlueBlock(blocks);
    } else if (strategy == PickupStrategy.ANY) {
      Block largestRed = getLargestRedBlock(blocks);
      Block largestBlue = getLargestBlueBlock(blocks);
      
      switch (DriverStation.getAlliance()) {
        default:
        case Red:
          // If alliance is red, prioritize red targets if they are there;
          // otherwise get blue targets
          latestTargetBall = largestRed == null ? largestBlue : largestRed;
          break;
        case Blue:
          // If alliance is blue, prioritize blue targets if they are there;
          // otherwise get red targets
          latestTargetBall = largestBlue == null ? largestRed : largestBlue;
          break;
        }
      } else if (strategy == PickupStrategy.OURS) {
        Block largestRed = getLargestRedBlock(blocks);
        Block largestBlue = getLargestBlueBlock(blocks);

      switch (DriverStation.getAlliance()) {
        default:
        case Red:
          // If alliance is red, prioritize red targets if they are there;
          // otherwise get blue targets
          latestTargetBall = largestRed;
          break;
        case Blue:
          // If alliance is blue, prioritize blue targets if they are there;
          // otherwise get red targets
          latestTargetBall = largestBlue;
          break;
        }
      } else if (strategy == PickupStrategy.THEIRS) {
        Block largestRed = getLargestRedBlock(blocks);
        Block largestBlue = getLargestBlueBlock(blocks);

      switch (DriverStation.getAlliance()) {
        default:
        case Red:
          // If alliance is red, prioritize red targets if they are there;
          // otherwise get blue targets
          latestTargetBall = largestBlue;
          break;
        case Blue:
          // If alliance is blue, prioritize blue targets if they are there;
          // otherwise get red targets
          latestTargetBall = largestRed;
          break;
      }
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

    // The first time we see a ball, we should turn the intake on
    // intake.start();

    double frameCenter = getFrameCenter();
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
    // intake.stop();
    targetBall = null;
  }

  @Override
  public State calculate(double forward, double strafe, boolean isFieldOriented) {
    if (targetBall != null) {
      return new AutoDrive.State(
        spangle,
        strafeOutput
      );
    } else {
      return null;
    }
  }

  public boolean isConnected() {
    return state == 0;
  }

  public double getFrameCenter() {
    return getFrameWidth() / 2.0;
  }

  public double getFrameWidth() {
    return (double) getFrameWidth();
  }

  public ArrayList<Block> getBlocks() {
    if (!isConnected()) {
      return new ArrayList<Block>();
    }

    // Either number of targets or an error code
    // Be careful changing the number at the end
    // We were having OutOfMemory errors at 20 and we belive a "safe" range is 4-8

    /*    Potential to add just a single signature instead of all 7 add PickupStrategy pickup to the method. would need to move the calls into the strategy block at 145
    if (pickup == PickupStrategy.RED) {
        // Either SIG1 or SIG2
      int error = pixycam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 2);
    } else {
      // Either SIG1 or SIG2
      int error = pixycam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 2);
    }
    */
    int error = pixycam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG_ALL, 2);

    if (error < 0) {
      return new ArrayList<Block>();
    }

    return pixycam.getCCC().getBlockCache();
  }


  public static Block getLargestRedBlock(ArrayList<Block> blocks) {
    return getLargestBlock(BlockSignature.RED, blocks);
  }

  public static Block getLargestBlueBlock(ArrayList<Block> blocks) {
    return getLargestBlock(BlockSignature.BLUE, blocks);
  }

  private static Block getLargestBlock(BlockSignature signature, ArrayList<Block> blocks) {
    Block largestBlock = null;

    for (Block block : blocks) {
      // Get ratio of width to height - looking for perfect squares to identify balls
      double ratio = block.getWidth() / block.getHeight();

      // Take reciprocal if greater than 1 for simplication purposes
      if (ratio > 1) {
        ratio = 1 / ratio;
      }

      // +/- 0.2 tolerance on "perfect square" to detect balls
      if (Utilities.withinTolerance(1.0, ratio, Constants.Pixy.RATIO_TOLERANCE)) {
        int blockSignature = block.getSignature();
        if (signature.value == blockSignature) {
          if (shouldUpdateLargestTarget(largestBlock, block)) {
            largestBlock = block;
          }
        }
      }
    }
    return largestBlock;
  }

  private static boolean shouldUpdateLargestTarget(Block largestBlock, Block newBlock) {
    if (largestBlock == null) {
      return true;
    }
    double largestBlockArea = largestBlock.getWidth() * largestBlock.getHeight();
    double newBlockArea = newBlock.getWidth() * newBlock.getHeight();
    return newBlockArea > largestBlockArea;
  }

}
