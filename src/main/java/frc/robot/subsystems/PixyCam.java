package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.ArrayList;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

/**
 * The code for retrieving information from the PixyCam using the SPI port
 * <p>
 * TODO: this file needs a refactor.
 * When creating this class originally, I just added a bunch of methods I thought
 * would be useful and ways to make them useful.
 * 
 * @author Michael Francis, Nicholas Stokes
 */
public class PixyCam extends SubsystemBase {

  // Variables for PixyCam
  private Pixy2 pixycam;
  private int state;
  private int chip;
  private int numberOfTargets;
  private boolean connected;
  private boolean seesTarget;
  private boolean retrievedState;

  private int cacheNumber;
  private int lastLargestBlockRetrieval;
  private Block lastLargestBlock;

  /**
   * Subsystem for the PixyCam
   * @param chipselect The chip the pixy is plugged into on the SPI
   */
  public PixyCam(int chipselect) {
    // Create a link
    pixycam = Pixy2.createInstance(Pixy2.LinkType.SPI);
    state = pixycam.init(chipselect);

    // Initialize variables
    connected = (state >= 0);
    chip = chipselect;
    seesTarget = false;
    retrievedState = false;
    cacheNumber = 0;
    lastLargestBlockRetrieval = -1;
    numberOfTargets = 0;

    // Prepare Shuffleboard stuff
    ShuffleboardTab pixyTab = Shuffleboard.getTab("PixyCam");

    ShuffleboardLayout infoWidget = pixyTab.getLayout("Vision Info", BuiltInLayouts.kList).withSize(4, 6).withPosition(4, 2);
    infoWidget.addNumber("Number of Targets", this::getNumberOfTargets);

    ShuffleboardLayout blockWidget = pixyTab.getLayout("Largest Block", BuiltInLayouts.kList).withSize(4, 8).withPosition(8, 0);
    Block largestBlock = getLargestTarget();

    // Hacky code to make sure we don't make method calls to things that don't exist
    blockWidget.addNumber("x pos", largestBlock != null ? largestBlock::getX : () -> -1);
    blockWidget.addNumber("y pos", largestBlock != null ? largestBlock::getY : () -> -1);
    blockWidget.addNumber("width", largestBlock != null ? largestBlock::getWidth : () -> -1);
    blockWidget.addNumber("height", largestBlock != null ? largestBlock::getHeight : () -> -1);
    blockWidget.addNumber("angle", largestBlock != null ? largestBlock::getAngle : () -> -1);
    blockWidget.addString("toString", largestBlock != null ? largestBlock::toString : () -> "");

    pixyTab.addBoolean("Sees Target", () -> seesTarget).withSize(2, 2).withPosition(4, 0);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {

    //DO NOT REMOVE THIS LINE OF CODE EVER
    updateTargets();

    //Check to see if the camera initialized correctly.
    if(!connected && !retrievedState) {
      //If we got here, the camera gave us an error.
      //Try to reinitialize the Pixy.
      state = pixycam.init(chip);
      //If we get another error, don't check again.
      //Keep the error code for reference
      if(state < 0) retrievedState = true;
    }

    //Detect connection
    connected = (state >= 0);
  }

  /**
   * Refreshes the target cache.
   */
  private void updateTargets() {
    // If the Pixy is returning an error, don't update the targets.
    if(state < 0) return;

    // Retrieve the targets and store the number in a variable
    numberOfTargets = pixycam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 48);

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
   * @return Returns the x-coordinate of the largest target from 0-315. 
   * Returns -1 if there isn't a target.
   */
  public int getLargestTargetX() {
    // Get the largest target
    Block largestTarget = getLargestTarget();
    // Return -1 if there was no target
    if(largestTarget == null)
      return -1;
    
    // Return the requested value
    return largestTarget.getX();
  }

  /**
   * @return Returns the y-coordinate of the largest target from 0-207. 
   * Returns -1 if there isn't a target.
   */
  public int getLargestTargetY() {
    // Get the largest target
    Block largestTarget = getLargestTarget();
    // Return -1 if there was no target
    if(largestTarget == null)
      return -1;
    
    // Return the requested value
    return largestTarget.getY();
  }

  /**
   * @return Returns the angle to the largest target in degrees from the center of the camera.
   * Ranges from -30 to 30. Returns 0.0 if no target was found.
   */
  public double getLargestTargetAngle() {
    double x = getLargestTargetX();
    // Return 0 (centered) if no target was found
    if(!seesTarget)
      return 0.0;
    
    /**
     * To get the angle, we divide the x (which ranges from 0 to 315, the width
     * of the camera) by 315 to get it as a percentage from 0-1. We multiply
     * that by 60 (the field of view of the PixyCam) to get it in terms of
     * degrees, and then subtract it by 30 to center it.
     */
    return ((x / 315) * 60) - 30;
  }

  /**
   * @return Returns the width of the largest target.
   */
  public int getLargestTargetWidth() {
    Block largestTarget = getLargestTarget();
    // Return -1 if there was no target
    if(largestTarget == null)
      return -1;
    
    // Return the requested value
    return largestTarget.getWidth();
  }

  /**
   * @return Returns the height of the largest target.
   * This is generally going to be smaller than the width because of lighting.
   */
  public int getLargestTargetHeight() {
    Block largestTarget = getLargestTarget();
    // Return -1 if there was no target
    if(largestTarget == null)
      return -1;
    
    //Return the requested value
    return largestTarget.getHeight();
  }
}