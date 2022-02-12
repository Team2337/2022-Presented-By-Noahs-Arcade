package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class CTREUtils {

  /**
   * Creates a current limit configuration with some generic values.
   * 
   * @return The generic current limit configuration
   */
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

  /**
   * Creates a custom current limit configuration. Equivalent to calling the constructor, but this
   * one actually has a Javadoc explaining the parameters. All units relating to current are in
   * amperes.
   * 
   * @param enable Whether or not the current limit feature is enabled.
   * @param currentLimit The "holding" current to limit when the feature is enabled
   * @param triggerThresholdCurrent Current must exceed this value for the current limit to start
   * taking effect. If larger than <code>currentLimit</code>, that value will be used.
   * @param triggerThresholdTime How long the current must exceed the threshold before the current
   * limit starts taking effect.
   * 
   * @return The custom current limit configuration
   */
  public static StatorCurrentLimitConfiguration createCurrentLimit(boolean enable, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime) {
    return new StatorCurrentLimitConfiguration(enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
  }

}
