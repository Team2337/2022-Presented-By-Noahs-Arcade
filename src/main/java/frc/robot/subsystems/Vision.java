package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SystemsCheckPositions;
import frc.robot.nerdyfiles.vision.LimelightUtilities;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private enum LimelightKey {
    Pipeline("pipeline"),
    LEDMode("ledMode"),
    X("tx"),
    Y("ty"),
    LATENCY("tl"),
    VALID_TARGET("tv");

    private String key;

    private LimelightKey(String key) {
      this.key = key;
    }
  }

  public static enum Pipeline {
    DEFAULT(1);

    private int number;

    private Pipeline(int number) {
      this.number = number;
    }

    public static Pipeline withNumber(int number) {
      for (Pipeline pipeline : Pipeline.values()) {
        if (pipeline.number == number) {
          return pipeline;
        }
      }
      return null;
    }
  }

  public static enum LEDMode {
    PIPELINE(0),
    OFF(1),
    BLINK(2),
    ON(3);

    private int value;

    private LEDMode(int value) {
      this.value = value;
    }

    public static LEDMode withValue(int value) {
      for (LEDMode mode : LEDMode.values()) {
        if (mode.value == value) {
          return mode;
        }
      }
      return null;
    }
  }

  // Only fetch our LL values once per periodic cycle
  private Pipeline currentPipeline;
  private LEDMode currentLEDMode;
  private double tx = 0.0;
  private double ty = 0.0;
  private double latency = 0.0;
  private boolean hasValidTarget = false;
  private double distanceToTargetMeters = 0.0;

  public int relocalizeCounter = 0;

  public Vision() {
    // Automatically switch our Limelight to our default pipeline on construction
    switchPipeLine(Pipeline.DEFAULT);

    // Systems check
    ShuffleboardTab systemsCheck = Shuffleboard.getTab("SYSTEMS CHECK");
    
    systemsCheck.addBoolean("Limelight", () -> (latency > 0))
      .withSize(2, 2)
      .withPosition(SystemsCheckPositions.LIMELIGHT.x, SystemsCheckPositions.LIMELIGHT.y);
  }

  @Override
  public void periodic() {
    currentPipeline = Pipeline.withNumber(getIntValue(LimelightKey.Pipeline));
    currentLEDMode = LEDMode.withValue(getIntValue(LimelightKey.LEDMode));
    tx = getDoubleValue(LimelightKey.X);
    ty = getDoubleValue(LimelightKey.Y);
    latency = getDoubleValue(LimelightKey.LATENCY);
    hasValidTarget = getDoubleValue(LimelightKey.VALID_TARGET) == 1.0;

    distanceToTargetMeters = 0.0;
    if (hasValidTarget) {
      distanceToTargetMeters = calculateDistanceToTargetMeters();
    }

    log();
  }

  private void log() {
    Logger.getInstance().recordOutput("Vision/# of relocalization", relocalizeCounter);
    Logger.getInstance().recordOutput("Vision/tx", getTx());
    Logger.getInstance().recordOutput("Vision/ty", getTy());
    Logger.getInstance().recordOutput("Vision/latency", getLatency());
    Logger.getInstance().recordOutput("Vision/Valid Target", hasActiveTarget());
    Logger.getInstance().recordOutput("Vision/Distance To Target (inches)", Units.metersToInches(distanceToTargetMeters));
  }

  /** Limelight Network Table Access */

  private static NetworkTableEntry getLimelightEntry(LimelightKey key) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(key.key);
  }

  /**
   * Get the double value for a key from the Limelight NetworkTables.
   *
   * @param key - The Limelight NetworkTables key
   * @return - the double value for the key in the Limelight NetworkTables. 0.0 if
   *         the key does not exist.
   */
  private static double getDoubleValue(LimelightKey key) {
    return getLimelightEntry(key).getDouble(0);
  }


  /**
   * Get the int value for a key from the Limelight NetworkTables.
   *
   * @param key - The Limelight NetworkTables key
   * @return - the int value for the key in the Limelight NetworkTables. 0 if
   *         the key does not exist.
   */
  private static int getIntValue(LimelightKey key) {
    return getLimelightEntry(key).getNumber(0).intValue();
  }

  /**
   * Sets the Limelight entry value.
   *
   * @param value the value to set
   * @return False if the entry exists with a different type
   */
  private static boolean setValue(LimelightKey key, Number value) {
    return getLimelightEntry(key).setNumber(value);
  }

  /** Limelight API */

  /**
   * Sets the LED mode to on, off, or blink
   */
  public void setLEDMode(LEDMode mode) {
    if (currentLEDMode != mode) {
      if (setValue(LimelightKey.LEDMode, mode.value)) {
        currentLEDMode = mode;
      }
    }
  }

  /**
   * Gets the Limelight LED mode
   */
  public LEDMode getLEDMode() {
    return currentLEDMode;
  }

  /**
   * Sets the pipeline of the Limelight
   */
  public void switchPipeLine(Pipeline pipeline) {
    if (currentPipeline != pipeline) {
      if (setValue(LimelightKey.Pipeline, pipeline.number)) {
        currentPipeline = pipeline;
      }
    }
  }

  /**
   * Gets the current pipeline on the Limelight
   * @return - Double value Limelight pipeline (0 -> 9)
   */
  public Pipeline getPipeline() {
    return currentPipeline;
  }

  public double getTx() {
    return tx;
  }

  public double getTy() {
    return ty;
  }

  public double getLatency() {
    return latency;
  }

  public boolean hasActiveTarget() {
    return hasValidTarget;
  }

  public double getDistanceToCenterHubMeters() {
    return distanceToTargetMeters + Constants.Vision.VISION_TARGET_OFFSET_FROM_HUB_CENTER_METERS;
  }

  public void incrementRelocalizeCounter() {
    relocalizeCounter++;
  }

  private double calculateDistanceToTargetMeters() {
    return LimelightUtilities.calculateDistanceToTargetMeters(
      Constants.getInstance().LIMELIGHT_CAMERA_HEIGHT_METERS,
      Constants.HUB_HEIGHT_METERS,
      Constants.getInstance().LIMEILGHT_CAMERA_ANGLE,
      Rotation2d.fromDegrees(getTy()),
      Rotation2d.fromDegrees(getTx())
    );
  }

}
