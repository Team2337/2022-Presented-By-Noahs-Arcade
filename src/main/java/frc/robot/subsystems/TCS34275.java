package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Code for the TCS34725 color sensor.
 * 
 * <p>
 * To find information about the hardware of the sensor, visit the following link and see pages 13-19:
 * https://cdn-shop.adafruit.com/datasheets/TCS34725.pdf
 * 
 * @author Michael F
 */
public class TCS34275 {
  
  private static final byte DEVICE_ADDRESS = 0x29;

  private I2C device;
  // private SimDevice simDevice;

  public enum Register {
    kEnable(0x00),
    kRGBCTime(0x01),
    kWaitTime(0x03),
    kClearInterruptLowThresholdLowByte(0x04),
    kClearInterruptLowThresholdHighByte(0x05),
    kClearInterruptHighThresholdLowByte(0x06),
    kClearInterruptHighThresholdHighByte(0x07),
    kInterruptPersistenceFilter(0x0C),
    kConfiguration(0x0D),
    kControl(0x0F),
    kDeviceID(0x12),
    kDeviceStatus(0x13),
    kClearDataLowByte(0x14),
    kClearDataHighByte(0x15),
    kRedDataLowByte(0x16),
    kRedDataHighByte(0x17),
    kGreenDataLowByte(0x18),
    kGreenDataHighByte(0x19),
    kBlueDataLowByte(0x1A),
    kBlueDataHighByte(0x1B);

    public final int bVal;

    Register(int i) {
      this.bVal = /* (byte) */ i;
    }
  }

  /**
   * A wrapper for the values of the color sensor
   */
  public static class RawColor {
    public final int clear, red, green, blue;
    public final double luminance;
    public RawColor(int clear, int red, int green, int blue, double luminance) {
      this.clear = clear;
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.luminance = luminance;
    }
  }

  public TCS34275(I2C.Port port) {
    device = new I2C(port, DEVICE_ADDRESS);
    initializeDevice();
  }

  private void initializeDevice() {
		// Select enable register
		// Power ON, RGBC enable, wait time disable
		device.write(0x80, 0x03);
		// Select ALS time register
		/**
     * According to the documentation, the values for the second parameter here are:
     * 
     * |------|-------|
     * | Time | Value |
     * |------|-------|
     * | 2.4  | 0xFF  |
     * | 24   | 0xF6  | <--
     * | 101  | 0xD5  |
     * | 154  | 0xC0  |
     * | 700  | 0x00  |
     * |------|-------|
     * 
     * Time values are in ms.
     */
		device.write(0x81, 0xF6);
		// Select Wait Time register
		// WTIME : 2.4ms
		device.write(0x83, 0xFF);
		// Select control register
		// AGAIN = 1x
		device.write(0x8F, 0x00);
  }

  private RawColor readData() {
		// Read 8 bytes of data
		// cData lsb, cData msb, red lsb, red msb, green lsb, green msb, blue lsb, blue msb
		byte[] data = new byte[8];
    device.read(0x94, 8, data);
		// Convert the data
		int clear = ((data[1] & 0xFF) * 256) + (data[0] & 0xFF);
		int red = ((data[3] & 0xFF) * 256) + (data[2] & 0xFF);
		int green = ((data[5] & 0xFF) * 256) + (data[4] & 0xFF); 
		int blue = ((data[7] & 0xFF) * 256) + (data[6] & 0xFF);
 
		// Calculate final lux
		double luminance = (-0.32466 * red) + (1.57837 * green) + (-0.73191 * blue);

    // Return new color
    return new RawColor(clear, red, green, blue, luminance);
  }

  public RawColor getRawColor() {
    return readData();
  }

  public Color getColor() {
    // Get raw data
    RawColor data = getRawColor();

    // Convert raw data to number between [0:1]
    // We normalize the vectors to mimic REV's color sensor
    double length = Math.sqrt(data.red*data.red + data.green*data.green + data.blue*data.blue);
    double red = (double)data.red / length;
    double green = (double)data.green / length;
    double blue = (double)data.blue / length;
    return new Color(red, green, blue);
  }
}
