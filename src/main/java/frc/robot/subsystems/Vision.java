package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for the vision system
 */
public class Vision extends SubsystemBase {

  /**
   * Logging
   */
  private Logger logger = Logger.getInstance();

  public Optional<Double> getLatency() {
    return getDoubleValue("tl");
  }

  /**
   * This will get the value from tx, ta, etc. by using a string
   *
   * @param key - string double value
   * @return - returns the double value from the string for example from ta, tx,
   *         etc.
   */
  private Optional<Double> getDoubleValue(String key) {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    if (limelightTable.containsKey(key)) {
      return Optional.of(limelightTable.getEntry(key).getDouble(0));
    }
    return Optional.empty();
  }

  @Override
  public void periodic() {
    Optional<Double> latency = getLatency();
    if (latency.isPresent()) {
      logger.recordOutput("Limelight/latency", latency.get());
    }
  }

}
