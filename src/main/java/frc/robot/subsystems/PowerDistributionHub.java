package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * @author Michael F
 */
public class PowerDistributionHub extends SubsystemBase {

  private PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  public PowerDistributionHub() {
    ShuffleboardTab pdhTab = Shuffleboard.getTab("PDH");
    ShuffleboardLayout widget = pdhTab.getLayout("Diagnostics", BuiltInLayouts.kList)
      .withSize(6, 12)
      .withPosition(0, 0);
    widget.addNumber("Temperature (F)", () -> {
      return (getTemperature() * (9.0/5.0)) + 32;
    });
    widget.addNumber("Voltage", this::getVoltage);
    widget.addNumber("Total Current (Amps)", pdh::getTotalCurrent);
    widget.addNumber("Total Energy (Joules)", pdh::getTotalEnergy);
    widget.addNumber("Total Power (Watts)", pdh::getTotalPower);
  }

  @Override
  public void periodic() {}

  // Switchable port

  /**
   * @param enabled Sets the switchable channel to be enabled/disabled
   */
  public void setSwitchablePort(boolean enabled) {
    pdh.setSwitchableChannel(enabled);
  }

  /**
   * @return Whether or not the switchable channel is disabled
   */
  public boolean getSwitchablePort() {
    return pdh.getSwitchableChannel();
  }

  // Getters for general info

  /**
   * @return Temperature of PDH in Celsius
   */
  public double getTemperature() {
    return pdh.getTemperature();
  }

  /**
   * @return Voltage of PDH
   */
  public double getVoltage() {
    return pdh.getVoltage();
  }

  // Individual channel currents

  /**
   * @param channel The channel to check the current of
   * @return The current of the specified channel in Amperes
   */
  public double getChannelCurrent(int channel) {
    return pdh.getCurrent(channel);
  }

  // TODO: these are examples, figure out if we want to have individual methods
  // for this or if we want to have a single method that gets the current
  public double getIntakeCurrent() {
    return getChannelCurrent(Constants.INTAKE_MOTOR_ID);
  }
  public double getDeliveryCurrent() {
    return getChannelCurrent(Constants.DELIVERY_MOTOR_ID);
  }

}
