package frc.robot.subsystems;

import frc.robot.Constants.BallColor;

public interface ColorSensor {

  /**
   * @return The currently viewed color, if any
   */
  public BallColor getColor();

  /**
   * @return Whether or not the sensor sees a ball, depending on certain conditions
   */
  public boolean seesBall();

}
