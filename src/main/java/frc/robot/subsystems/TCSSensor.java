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
 * <p>
 * Code was based on http://www.getmicros.net/raspberry-pi-and-tcs34725-color-sensor-java-example.php
 * 
 * @author Michael F
 */
public class TCSSensor {
  
  private static final byte DEVICE_ADDRESS = 0x29;

  private I2C device;

  public enum Register {
    // Registers are named as they are in the documentation.
    /** Enables states and interrupts */
    ENABLE(0x80),
    /** RGBC time */
    ATIME(0x81),
    /** Wait time */
    WTIME(0x83),
    /** Clear interrupt low threshold low byte */
    AILTL(0x84),
    /** Clear interrupt low threshold high byte */
    AILTH(0x85),
    /** Clear interrupt high threshold low byte */
    AIHTL(0x86),
    /** Clear interrupt high threshold high byte */
    AIHTH(0x87),
    /** Interrupt persistence filter */
    PERS(0x8C),
    /** Configuration */
    CONFIG(0x8D),
    /** Control */
    CONTROL(0x8F),
    /** Device ID */
    ID(0x92),
    /** Device status */
    STATUS(0x93),
    /** Clear data low byte */
    CDATAL(0x94),
    /** Clear data high byte */
    CDATAH(0x95),
    /** Red data low byte */
    RDATAL(0x96),
    /** Red data high byte */
    RDATAH(0x97),
    /** Green data low byte */
    GDATAL(0x98),
    /** Green data high byte */
    GDATAH(0x99),
    /** Blue data low byte */
    BDATAL(0x9A),
    /** Blue data high byte */
    BDATAH(0x9B);

    public final int value;

    Register(int i) {
      this.value = /* (byte) */ i;
    }
  }

  /**
   * A wrapper for the values of the color sensor
   */
  public static class TCSColor {
    public final int clear, red, green, blue;
    public final double luminance;
    public TCSColor(int clear, int red, int green, int blue, double luminance) {
      this.clear = clear;
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.luminance = luminance;
    }
  }

  public TCSSensor(I2C.Port port) {
    device = new I2C(port, DEVICE_ADDRESS);
    initializeDevice();
  }

  private void initializeDevice() {
    // Select enable register
    // Power ON, RGBC enable, wait time disable
    device.write(Register.ENABLE.value, 0x03);
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
     * Time values are in ms. See table 6 in PDF.
     */
    device.write(Register.ATIME.value, 0xF6);
    // Select control register
    /**
     * According to the documentation, the values for the second parameter here are:
     * 
     * |------|-------|
     * | Gain | Value |
     * |------|-------|
     * | 1x   | 0b00  |
     * | 4x   | 0b01  |
     * | 16x  | 0b10  | <--
     * | 60x  | 0b11  |
     * |------|-------|
     * 
     * See table 11 in PDF.
     */
    device.write(Register.CONTROL.value, 0b10);
  }

  private TCSColor readData() {
    // Read 8 bytes of data
    // cData lsb, cData msb, red lsb, red msb, green lsb, green msb, blue lsb, blue msb
    byte[] data = new byte[8];
    device.read(Register.CDATAL.value, 8, data);
    // Convert the data
    //           Register.CDATAH           Register.CDATAL
    int clear = ((data[1] & 0xFF) << 8) + (data[0] & 0xFF);
    //           Register.RDATAH           Register.RDATAL
    int red =   ((data[3] & 0xFF) << 8) + (data[2] & 0xFF);
    //           Register.GDATAH           Register.GDATAL
    int green = ((data[5] & 0xFF) << 8) + (data[4] & 0xFF);
    //           Register.BDATAH           Register.BDATAL 
    int blue =  ((data[7] & 0xFF) << 8) + (data[6] & 0xFF);
 
    // Calculate final lux
    double luminance = (-0.32466 * red) + (1.57837 * green) + (-0.73191 * blue);

    // Return new color
    return new TCSColor(clear, red, green, blue, luminance);
  }

  public TCSColor getRawColor() {
    return readData();
  }

  public Color getColor() {
    // Get raw data
    TCSColor data = getRawColor();

    /*
     * Convert raw data to number between [0:1].
     * 
     * To mimic the REV sensor, they need to add up to 1. To do this, we divide each number
     * by the sum. This preserves the ratios. This doesn't work with negative numbers, but
     * we don't use those so it doesn't matter.
     */
    double sum = (double)(data.red + data.green + data.blue);
    double red = (double)data.red / sum;
    double green = (double)data.green / sum;
    double blue = (double)data.blue / sum;
    return new Color(red, green, blue);
  }
}
