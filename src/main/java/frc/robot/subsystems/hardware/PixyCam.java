package frc.robot.subsystems.hardware;

import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.Utilities;

import java.util.ArrayList;

import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

/**
 * The code for retrieving information from the PixyCam using the SPI port
 */
public class PixyCam {

  private enum BlockSignature {
    RED(1),
    BLUE(2);

    int value;

    BlockSignature(int value) {
      this.value = value;
    }
  }

  private final Pixy2 pixycam = Pixy2.createInstance(Pixy2.LinkType.SPI);
  private int state;

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
    state = pixycam.init(chipselect);
  }

  public boolean isConnected() {
    return state == 0;
  }

  public double getFrameWidth() {
    return (double) pixycam.getFrameWidth();
  }

  public double getFrameCenter() {
    return getFrameWidth() / 2.0;
  }

  public ArrayList<Block> getBlocks() {
    if (!isConnected()) {
      return new ArrayList<Block>();
    }

    // Either number of targets or an error code
    // Be careful changing the number at the end
    // We were having OutOfMemory errors at 20 and we belive a "safe" range is 4-8
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