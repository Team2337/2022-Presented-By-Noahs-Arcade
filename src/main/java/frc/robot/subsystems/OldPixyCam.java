package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PixySigs;
import edu.wpi.first.wpilibj.shuffleboard.*;
import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

/**
 * The (old) code for retrieving information from the PixyCam using the SPI port
 * 
 * @author Michael Francis, Nicholas Stokes
 * 
 * @deprecated
 */
public class OldPixyCam extends SubsystemBase {

  // Variables for PixyCam
  private Pixy2 pixycam;
  private int state;
  private int chip;
  private int numberOfTargets;
  private boolean connected;
  private boolean seesTarget;

  private int cacheNumber;
  private int lastLargestBlockRetrieval;
  private Block lastLargestBlock;

  /**
   * Subsystem for the PixyCam
   * @param chipselect The chip the pixy is plugged into on the SPI
   */
  public OldPixyCam(int chipselect) {
    // Create a link
    pixycam = Pixy2.createInstance(Pixy2.LinkType.SPI);
    state = pixycam.init(chipselect);

    // Initialize variables
    connected = (state >= 0);
    chip = chipselect;
    seesTarget = false;
    cacheNumber = 0;
    lastLargestBlockRetrieval = -1;
    numberOfTargets = 0;

    // Prepare Shuffleboard stuff
    ShuffleboardTab pixyTab = Shuffleboard.getTab("PixyCam");

    // Targetting red boolean
    pixyTab.addBoolean("Targeting red", () -> {
      return (numberOfTargets > 0 && lastLargestBlock != null) ? lastLargestBlock.getSignature() == 1 : false;
    })
      .withSize(4, 4)
      .withPosition(12, 0)
      .withProperties(Map.of("Color when true", "#ff6666"))
      .withProperties(Map.of("Color when false", "#000000"));
    // Targetting blue boolean
    pixyTab.addBoolean("Targeting blue", () -> {
      return (numberOfTargets > 0 && lastLargestBlock != null) ? lastLargestBlock.getSignature() == 2 : false;
    })
      .withSize(4, 4)
      .withPosition(12, 4)
      .withProperties(Map.of("Color when true", "#6666ff"))
      .withProperties(Map.of("Color when false", "#000000"));

    // Info widget
    ShuffleboardLayout infoWidget = pixyTab.getLayout("Vision Info", BuiltInLayouts.kList).withSize(4, 6).withPosition(4, 4);
    infoWidget.addNumber("Number of Targets", this::getNumberOfTargets);
    infoWidget.addString("Target list", () -> getAllTargets().toString());
    infoWidget.addNumber("Pixy State", () -> state);

    // Block info widget
    ShuffleboardLayout blockWidget = pixyTab.getLayout("Largest Block", BuiltInLayouts.kList).withSize(4, 8).withPosition(8, 0);
    blockWidget.addNumber("signature", () -> {
      return (numberOfTargets > 0 && lastLargestBlock != null) ? lastLargestBlock.getSignature() : 0;
    });

    pixyTab.addBoolean("Sees Target", () -> seesTarget)
      .withSize(4, 4)
      .withPosition(4, 0);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {

    // The following line of code is very important.
    updateTargets();
    getLargestTarget();

    // Check to see if the camera initialized correctly.
    if(!connected) {
      // If we got here, the camera gave us an error.
      // Try to reinitialize the Pixy.
      state = pixycam.init(chip);
    }

    // Detect connection
    connected = (state >= 0);
  }

  /**
   * Refreshes the target cache.
   */
  private void updateTargets() {
    // If the Pixy is returning an error, don't update the targets.
    if(state < 0) return;

    // Retrieve the targets and store the number in a variable
    int error = pixycam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG_ALL, 48);
    if(error < 0) return;
    numberOfTargets = error;

    // Update variables
    cacheNumber++;
    seesTarget = numberOfTargets > 0;
  }

  /**
   * @return The number of targets in view of the camera (or the last number retrieved)
   */
  public int getNumberOfTargets() {
    return numberOfTargets;
  }

  /**
   * Gets all cached targets. Be sure to update it with updateTargets()
   * @return An ArrayList of target data.
   */
  public ArrayList<Block> getAllTargets() {
    // Retrieve all blocks
    return pixycam.getCCC().getBlockCache();
  }

  /**
   * Gets the largest target.
   * @return A Block class containing the largest target.
   * @see Block
   */
  public Block getLargestTarget() {
    // See if we already have the largest Block (to be efficient)
    if(lastLargestBlockRetrieval == cacheNumber) {
      return lastLargestBlock;
    }

    // Check to see if there are any targets.
    if(!seesTarget) return null;

    // Get all the targets
    ArrayList<Block> blocks = getAllTargets();
    Block largestBlock = null;
    // Loops through all targets and finds the widest one
    for(Block block : blocks) {
      if(largestBlock == null) {
        // If this is the first iteration, set largestBlock to the current block.
        largestBlock = block;
      } else if(block.getWidth() > largestBlock.getWidth()) {
        // If we find a wider block, set largestBlock to the current block.
        largestBlock = block;
      }
    }

    // Update the last time we looked for the largest Block
    lastLargestBlockRetrieval = cacheNumber;
    // Store this Block
    lastLargestBlock = largestBlock;

    // Return the Blocks
    return largestBlock;
  }

  /**
   * @return Returns the angle to the largest target in degrees from the center of the camera.
   * Ranges from -30 to 30. Returns empty optional if no target was found.
   */
  public Optional<Double> getLargestTargetAngle() {
    double x = lastLargestBlock.getX();
    // Return 0 (centered) if no target was found
    if(!seesTarget)
      return Optional.empty();
    
    /**
     * To get the angle, we divide the x (which ranges from 0 to 315, the width
     * of the camera) by 315 to get it as a percentage from 0-1. We multiply
     * that by 60 (the field of view of the PixyCam) to get it in terms of
     * degrees, and then subtract it by 30 to center it.
     */
    return Optional.of(((x / 315) * 60) - 30);
  }

  public PixySigs getLargestTargetColor() {
    if(lastLargestBlock == null)
      return PixySigs.None;
    
    // According to programmed values on robot
    return lastLargestBlock.getSignature() == 1 ? PixySigs.Red : PixySigs.Blue;
  }
}