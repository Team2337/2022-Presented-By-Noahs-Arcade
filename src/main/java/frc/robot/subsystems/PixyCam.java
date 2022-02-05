package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Map;
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

  // 

  // private ArrayList<Block> blocks = new ArrayList<Block>();

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

    // Prepare Shuffleboard stuff
    ShuffleboardTab pixyTab = Shuffleboard.getTab("PixyCam");

    // Targetting red boolean
    pixyTab.addBoolean("Targeting red", () -> (redTarget != null))
      .withSize(4, 4)
      .withPosition(12, 0)
      .withProperties(Map.of("Color when true", "#ff6666"))
      .withProperties(Map.of("Color when false", "#000000"));
    // Targetting blue boolean
    pixyTab.addBoolean("Targeting blue", () -> (blueTarget != null))
      .withSize(4, 4)
      .withPosition(12, 4)
      .withProperties(Map.of("Color when true", "#6666ff"))
      .withProperties(Map.of("Color when false", "#000000"));

    // Info widget
    ShuffleboardLayout infoWidget = pixyTab.getLayout("Vision Info", BuiltInLayouts.kList).withSize(8, 6).withPosition(4, 4);
    // infoWidget.addNumber("Number of Blocks", () -> blocks.size());
    // infoWidget.addNumber("Number of Targets", () -> filteredBlocks.size());
    infoWidget.addNumber("Red target x", () -> {
      return redTarget == null ? -1 : redTarget.getX();
    });
    infoWidget.addNumber("Red target y", () -> {
      return redTarget == null ? -1 : redTarget.getY();
    });
    infoWidget.addString("Red target angle", () -> {
      return redTarget == null ? "" : String.valueOf(getTargetAngle(redTarget).get());
    });
    infoWidget.addNumber("Blue target x", () -> {
      return blueTarget == null ? -1 : blueTarget.getX();
    });
    infoWidget.addNumber("Blue target y", () -> {
      return blueTarget == null ? -1 : blueTarget.getY();
    });
    infoWidget.addString("Blue target angle", () -> {
      return blueTarget == null ? "" : String.valueOf(getTargetAngle(blueTarget).get());
    });
    infoWidget.addNumber("Pixy State", () -> state);

    // pixyTab.addBoolean("Sees Target", () -> (blocks.size() > 0))
    //   .withSize(4, 4)
    //   .withPosition(4, 0);
  }

  @Override
  public void periodic() {
    // Filter target values
    filterTargets(updatePixy());
  }

  /**
   * Updates the values of the PixyCam
   */
  private ArrayList<Block> updatePixy() {
    // Check to see if the camera initialized correctly.
    if(!connected) {
      // Attempt to reconnect Pixy
      state = pixycam.init(chip);
      connected = (state >= 0);
      if(!connected) return null;
    }

    // Either number of targets or an error code
    int error = pixycam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG_ALL, 20);

    // If there was an error (i.e. Pixy is busy getting new values), don't update variables
    if(error < 0)
      return null;

    return pixycam.getCCC().getBlockCache();
  }

  /**
   * Filters the targets based on conditions that make them seem "cargo-like"
   */
  private void filterTargets(ArrayList<Block> blocks) {
    if (blocks == null) {
      return;
    }
    
    // Update entries
    redTarget = null;
    blueTarget = null;

    /**
     * TODO: get blocks returns largest blocks first, adapt code
     */

    ArrayList<Block> filteredBlocks = new ArrayList<Block>();

    for(Block block : blocks) {
      // Get ratio of width to height
      double ratio = block.getWidth() / block.getHeight();

      // Take reciprocal if less than 1 for simplication purposes
      if(ratio < 1)
        ratio = 1 / ratio;
      
      // Check if it matches conditions
      //FIXME: add proper ball detection
      if(ratio < Constants.PIXY_RATIO_THRESHOLD) {
        // Add it to filtered list
        filteredBlocks.add(block);

        // Set red or blue target to whatever this one is
        double area = block.getWidth() * block.getHeight();

        if (block.getSignature() == 1) {
          if (redTarget == null || redTarget.getWidth() * redTarget.getHeight() < area) {
            redTarget = block;
          }
        } else {
          if (blueTarget == null || blueTarget.getWidth() * blueTarget.getHeight() < area) {
            blueTarget = block;
          }
        }
      }
    }
  }

  /**
   * <b>Note:</b> this will probably be replaced with a way to specify a trajectory
   * 
   * @return The largest red target that seems "cargo-like"
   */
  public Block getRedTarget() {
    return redTarget;
  }

  /**
   * <b>Note:</b> this will probably be replaced with a way to specify a trajectory
   * 
   * @return The largest blue target that seems "cargo-like"
   */
  public Block getBlueTarget() {
    return blueTarget;
  }

  /**
   * @return Whether or not the Pixy sees a red cargo
   */
  public boolean seesRedTarget(){
    return getRedTarget() != null;
  }

  /**
   * @return Whether or not the Pixy sees a blue cargo
   */
  public boolean seesBlueTarget(){
    return getBlueTarget() != null;
  }

  /**
   * @param target The target {@link Block}
   * @return The target converted to an angle from center of Pixy.
   * Ranges from -30 to 30. Returns empty if target is null.
   */
  public Optional<Double> getTargetAngle(Block target) {
    if (target == null){
      return Optional.empty();
    }

    /**
     * To get the angle, we divide the x (which ranges from 0 to 315, the width
     * of the camera) by 315 to get it as a percentage from 0-1. We multiply
     * that by 60 (the field of view of the PixyCam) to get it in terms of
     * degrees, and then subtract it by 30 to center it.
     */
    return Optional.of(((target.getX() / 315.0) * 60.0) - 30.0);
  }

  public double getFrameWidth() {
    return pixycam.getFrameWidth();
  }

}