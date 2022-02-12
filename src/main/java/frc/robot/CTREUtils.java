package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class CTREUtils {

  /**
   * Creates a current limit configuration with some generic values.
   * 
   * @return The generic current limit configuration
   */
  public static StatorCurrentLimitConfiguration defaultCurrentLimit() {
    return new StatorCurrentLimitConfiguration(true, 50.0, 40.0, 3.0);
  }

}
