package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class CTREUtils {

  public static StatorCurrentLimitConfiguration configureDefaultCurrentLimit(TalonFX motor) {
    // TODO: these values were copied from shooter. What do we actually want for these?
    StatorCurrentLimitConfiguration configuration = new StatorCurrentLimitConfiguration();
    configuration.enable = true;
    configuration.currentLimit = 50;
    configuration.triggerThresholdCurrent = 40;
    configuration.triggerThresholdTime = 3;
    motor.configStatorCurrentLimit(configuration);
    return configuration;
  }

}
