package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BallColor;
import frc.robot.nerdyfiles.utilities.Utilities;
import java.util.ArrayList;
import java.util.Map;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

/**
 * The code for retrieving information from the PixyCam using the SPI port
 *
 * @author Michael Francis
 */
public class PixyCam extends SubsystemBase {

  public class Ball {
    public final int x, y;
    public final BallColor color;

    Ball(int x, int y, BallColor color) {
      this.x = x;
      this.y = y;
      this.color = color;
    }

    public double getAngle() {
      return ((x / getFrameWidth()) * 60.0) - 30.0;
    }
  }

  private final Pixy2 pixycam = Pixy2.createInstance(Pixy2.LinkType.SPI);;
  private final int chipselect;
  private int state;

  private Block largestRedTarget;
  private Block largestBlueTarget;

  /**
   * Creates a PixyCam connected on the SPI chipselect 0
   */
  public PixyCam() {
    this(0);
  }

  /**
   * Subsystem for the PixyCam
   * @param chipselect The chip the pixy is plugged into on the SPI
   */
  public PixyCam(int chipselect) {
    this.chipselect = chipselect;

    connect();

    setupShuffleboard();
  }

  private void setupShuffleboard() {
    ShuffleboardTab pixyTab = Shuffleboard.getTab("PixyCam");
    pixyTab.addBoolean("Targeting red", () -> (largestRedTarget != null))
      .withSize(4, 4)
      .withPosition(12, 0)
      .withProperties(Map.of("Color when true", "#ff6666"))
      .withProperties(Map.of("Color when false", "#000000"));
    pixyTab.addBoolean("Targeting blue", () -> (largestBlueTarget != null))
      .withSize(4, 4)
      .withPosition(12, 4)
      .withProperties(Map.of("Color when true", "#6666ff"))
      .withProperties(Map.of("Color when false", "#000000"));

    ShuffleboardLayout infoWidget = pixyTab.getLayout("Vision Info", BuiltInLayouts.kList).withSize(8, 6).withPosition(4, 4);
    infoWidget.addNumber("Red target x", () -> {
      return largestRedTarget == null ? -1 : largestRedTarget.getX();
    });
    infoWidget.addNumber("Red target y", () -> {
      return largestRedTarget == null ? -1 : largestRedTarget.getY();
    });
    infoWidget.addString("Red target angle", () -> {
      return largestRedTarget == null ? "" : String.valueOf(getTargetAngle(largestRedTarget));
    });
    infoWidget.addNumber("Blue target x", () -> {
      return largestBlueTarget == null ? -1 : largestBlueTarget.getX();
    });
    infoWidget.addNumber("Blue target y", () -> {
      return largestBlueTarget == null ? -1 : largestBlueTarget.getY();
    });
    infoWidget.addString("Blue target angle", () -> {
      return largestBlueTarget == null ? "" : String.valueOf(getTargetAngle(largestBlueTarget));
    });
    infoWidget.addNumber("Pixy State", () -> state);
  }

  private void connect() {
    state = pixycam.init(this.chipselect);
  }

  private boolean isConnected() {
    return state == 0;
  }

  @Override
  public void periodic() {
    // If we aren't connected, bail on doing stuff to avoid null pointer exceptions
    if (!isConnected()) {
      return;
    }

    // Clear our previous blocks in prep for new blocks
    largestRedTarget = null;
    largestBlueTarget = null;

    filterTargets(updatePixy());
  }

  /**
   * Updates the values of the PixyCam
   */
  private ArrayList<Block> updatePixy() {
    // Either number of targets or an error code
    int error = pixycam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG_ALL, 20);

    if (error < 0) {
      return new ArrayList<Block>();
    }

    return pixycam.getCCC().getBlockCache();
  }

  /**
   * Filters the targets based on conditions that make them seem "cargo-like"
   */
  private void filterTargets(ArrayList<Block> blocks) {
    for (Block block : blocks) {
      // Get ratio of width to height - looking for perfect squares to identify balls
      double ratio = block.getWidth() / block.getHeight();

      // Take reciprocal if greater than 1 for simplication purposes
      if (ratio > 1) {
        ratio = 1 / ratio;
      }

      // +/- 0.2 tolerance on "perfect square" to detect balls
      if (Utilities.withinTolerance(1.0, ratio, Constants.Pixy.RATIO_TOLERANCE)) {
        // Red == Block Signature 1, Blue == Block Signature 2
        int signature = block.getSignature();
        if (signature == 1) {
          if (shouldUpdateLargestTarget(largestRedTarget, block)) {
            largestRedTarget = block;
          }
        } else if (signature == 2) {
          if (shouldUpdateLargestTarget(largestBlueTarget, block)) {
            largestBlueTarget = block;
          }
        }
      }
    }
  }

  private static boolean shouldUpdateLargestTarget(Block largestBlock, Block newBlock) {
    if (largestBlock == null) {
      return true;
    }
    double largestBlockArea = largestBlock.getWidth() * largestBlock.getHeight();
    double newBlockArea = newBlock.getWidth() * newBlock.getHeight();
    return newBlockArea > largestBlockArea;
  }

  /**
   * @return The largest red target that seems "cargo-like"
   */
  public Block getRedTarget() {
    return largestRedTarget;
  }

  /**
   * @return The largest blue target that seems "cargo-like"
   */
  public Block getBlueTarget() {
    return largestBlueTarget;
  }

  /**
   * @return Whether or not the Pixy sees a red cargo
   */
  public boolean seesRedTarget() {
    return getRedTarget() != null;
  }

  /**
   * @return Whether or not the Pixy sees a blue cargo
   */
  public boolean seesBlueTarget() {
    return getBlueTarget() != null;
  }

  /**
   * @param target The target {@link Block}
   * @return The target converted to an angle from center of Pixy.
   * Ranges from -30 to 30. Returns empty if target is null.
   */
  public Double getTargetAngle(Block target) {
    if (target == null) {
      return null;
    }

    /**
     * To get the angle, we divide the x by the width of the camera to get it
     * as a percentage from 0-1. We multiply that by 60 (the horizontal field
     * of view of the PixyCam) to get it in terms of degrees, and then subtract
     * it by 30 to center it.
     */
    return ((target.getX() / getFrameWidth()) * 60.0) - 30.0;
  }

  public double getFrameWidth() {
    return (double)pixycam.getFrameWidth();
  }

}