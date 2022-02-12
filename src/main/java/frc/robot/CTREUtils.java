package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class CTREUtils {

  public static StatorCurrentLimitConfiguration configureDefaultCurrentLimit() {
    // TODO: these values were copied from shooter. What do we actually want for these?
    StatorCurrentLimitConfiguration configuration = new StatorCurrentLimitConfiguration();
    configuration.enable = true;
    configuration.currentLimit = 50;
    configuration.triggerThresholdCurrent = 40;
    configuration.triggerThresholdTime = 3;
    return configuration;
  }

}
