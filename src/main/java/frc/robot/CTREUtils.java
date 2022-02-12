package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class CTREUtils {

  public static StatorCurrentLimitConfiguration defaultCurrentLimit() {
    /**
     * TODO: What values do we actually want for these?
     * These values were copied from shooter, which seems to be copied from 2020 code.
     * https://github.com/Team2337/2020-Perpetual-Supercharger/blob/b4366826ed0e9842878486df3a2a3898ca411ebe/src/main/java/frc/robot/subsystems/Intake.java#L54-L57
     */
    StatorCurrentLimitConfiguration configuration = new StatorCurrentLimitConfiguration();
    configuration.enable = true;
    configuration.currentLimit = 50.0;
    configuration.triggerThresholdCurrent = 40.0;
    configuration.triggerThresholdTime = 3.0;
    return configuration;
  }

  public static StatorCurrentLimitConfiguration createCurrentLimit(boolean enable, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime) {
    return new StatorCurrentLimitConfiguration(enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
  }

}
