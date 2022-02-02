package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private boolean rotateLimelight = false;
  private boolean visionDebug = false;

  private ShuffleboardTab tab = Shuffleboard.getTab("Vision");

  private MedianFilter medianFilter = new MedianFilter(10);
  private double calculatedTx;

  public Vision() {
    ShuffleboardLayout dataWidget = tab.getLayout("Data", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);

    calculatedTx = getDoubleValue("tx");
    dataWidget.addNumber("tx (Median)", () -> calculatedTx);

    dataWidget.addNumber("ts", () -> getDoubleValue("ts"));
  }

  /**
   * Sets the LED mode to on, off, or blink
   * @param mode - the mode of the LEDs
   * Example:
   * 0: Sets the mode to what is in the current pipeline
   * 1: Turns off the LEDs
   * 2: Blink mode on LEDs
   * 3: Turns on the LEDs
   */
  public void setLEDMode(int mode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
  }

  /**
   * Gets the Limelight mode number from the NetworkTable
   * @return - returns the mode number from the NetworkTable
   */
  public int getLEDMode() {
    return (int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getValue().getDouble();
  }

  /**
   * Sets the pipeline of the limelight
   * @param pipeline - sets the desired pipeline number between 0-9
   * 0 - CloseVision (0-15 ft away from the tower)
   * 1 - MediumVision (between 15-21 ft)
   * 2 - FarVision (21-26 ft)
   * 9 - Drivecam
   */
  public void switchPipeLine(int pipeline) {
    double currentPipeline = NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0);
    if (currentPipeline != pipeline) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }
  }

  /**
   * Gets the current pipeline on the limelight
   * @return - Double value limelight pipeline (0 -> 9)
   */
  public double getPipeline() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0);
  }

  public double getTx() {
    return calculatedTx;
  }

  // TODO: Rename
  public boolean getTv() {
    return (int)getDoubleValue("tv") == 1.0 ? true : false;
  }

  public double getTy() {
    return getDoubleValue("ty");
  }

  /**
   * This will get the value from tx, ta, etc. by using a string
   * @param output - string double value
   * @return - returns the double value from the string for example from ta, tx, etc.
   */
  private double getDoubleValue(String output) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(output).getDouble(0);
  }

  /**
   * This will get the X coordinte from Opensight
   * @return - returns the x-coordinate value, which will show how far we need to rotate
   */
  public double getOpenSightXCoordinateValue() {
    return NetworkTableInstance.getDefault().getTable("PutCoordinate").getEntry("coord-x").getDouble(0);
  }

  /**
   * This will get the Y coordinate from Opensight
   * @return - returns the y-coordinate value, which will show how far the robot needs to drive
   */
  public double getOpenSightYCoordinateValue() {
    return NetworkTableInstance.getDefault().getTable("PutCoordinate").getEntry("coord-y").getDouble(0);
  }

  /**
   * Receives the NetworkTable values from Opensight
   * @return - returns the opensight networktable which will return a true/false value
   */
  public boolean getOpenSightNTValue() {
    return NetworkTableInstance.getDefault().getTable("PutNT").getEntry("succ").getBoolean(false);
  }

  /**
   * Lets us know if we are in the limelight mode, we are rotating using the limelight
   * @param rotateLimelight - Boolean value (limelight mode: true | not limelight mode: false)
   */
  public void setRotateLimelight(boolean rotateLimelight) {
    this.rotateLimelight = rotateLimelight;
  }

  /**
   * Lets us know if we are in the limelight mode, we are rotating using the limelight
   * @return - Boolean value limelight mode
   */
  public boolean getRotateLimelight() {
    return rotateLimelight;
  }

  @Override
  public void periodic() {
    calculatedTx = medianFilter.calculate(getDoubleValue("tx"));
  }
}