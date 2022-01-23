package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Optional;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

/**
 * The code for retrieving information from the PixyCam using the SPI port
 * 
 * @author Michael Francis
 */
public class PixyCam extends SubsystemBase {

  // Variables for PixyCam
  final private Pixy2 pixycam;
  final private int chip;
  private int state;
  private boolean connected;

  private int numberOfTargets;
  private ArrayList<Block> blocks;
  private ArrayList<Block> filteredBlocks;

  private Block redTarget;
  private Block blueTarget;

  /**
   * Subsystem for the PixyCam
   * @param chipselect The chip the pixy is plugged into on the SPI
   */
  public PixyCam(int chipselect) {
    // Create a link
    pixycam = Pixy2.createInstance(Pixy2.LinkType.SPI);
    state = pixycam.init(chipselect);

    // Initialize variables
    chip = chipselect;
    numberOfTargets = 0;
  }

  @Override
  public void periodic() {
    // Update Pixy values
    updatePixy();

    // Filter target values
    filterTargets();
  }

  /**
   * Updates the values of the PixyCam
   */
  private void updatePixy() {
    // Check to see if the camera initialized correctly.
    if(!connected) {
      // Attempt to reconnect Pixy
      state = pixycam.init(chip);
      connected = (state >= 0);
      if(!connected) return;
    }

    // Either number of targets or an error code
    int error = pixycam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG_ALL, 48);

    // If there was an error (i.e. Pixy is busy getting new values), don't update variables
    if(error < 0)
      return;

    numberOfTargets = error;
    blocks = pixycam.getCCC().getBlockCache();
  }

  /**
   * Filters the targets based on conditions that make them seem "cargo-like"
   */
  private void filterTargets() {
    // Skip the method if number of targets is 0
    if(numberOfTargets == 0)
      return;
    
    // Filter ArrayList
    Block bestRedBlock = null;
    Block bestBlueBlock = null;
    filteredBlocks.clear();

    for(Block block : blocks) {
      // Get ratio of width to height
      double ratio = block.getWidth() / block.getHeight();

      // Take reciprocal if less than 1 for simplication purposes
      if(ratio < 1)
        ratio = 1 / ratio;
      
      // Check if it matches conditions
      if(ratio < Constants.PIXY_RATIO_THRESHOLD) {
        // Add it to filtered list
        filteredBlocks.add(block);

        // Set red or blue target to whatever this one is
        double area = block.getWidth() * block.getHeight();
        if(block.getSignature() == 1){
          if(redTarget == null || redTarget.getWidth() * redTarget.getHeight() < area){
            redTarget = block;
          }
        } else {
          if(blueTarget == null || blueTarget.getWidth() * blueTarget.getHeight() < area){
            blueTarget = block;
          }
        }
      }
    }

    redTarget = bestRedBlock;
    blueTarget = bestBlueBlock;
  }

  /**
   * <b>Note:</b> this will probably be replaced with a way to specify a trajectory
   * 
   * @return The largest red target that seems "cargo-like"
   */
  public Optional<Block> getRedTarget() {
    if(redTarget == null)
      return Optional.empty();
    
    return Optional.of(redTarget);
  }

  /**
   * <b>Note:</b> this will probably be replaced with a way to specify a trajectory
   * 
   * @return The largest blue target that seems "cargo-like"
   */
  public Optional<Block> getBlueTarget() {
    if(blueTarget == null)
      return Optional.empty();
    
    return Optional.of(blueTarget);
  }

  /**
   * @return Whether or not the Pixy sees a red cargo
   */
  public boolean seesRedTarget(){
    return getRedTarget().isPresent();
  }

  /**
   * @return Whether or not the Pixy sees a blue cargo
   */
  public boolean seesBlueTarget(){
    return getBlueTarget().isPresent();
  }

}