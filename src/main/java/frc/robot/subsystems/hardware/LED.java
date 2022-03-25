package frc.robot.subsystems.hardware;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.Utilities;

/**
 * Setup for addressable LED strip
 * 
 * @author Madison J
 */

public class LED extends SubsystemBase {

	private static AddressableLED led;
	private static AddressableLEDBuffer ledBuffer;

	/**
	 * Controls the LEDs on the Robot 
	 * 
	 * @param pwm The PWM port that the blinkin is plugged into
	 */
	public LED() {
		led = new AddressableLED(Constants.LEDSTRIP_PWM_ID);
		ledBuffer = new AddressableLEDBuffer(16);
		led.setLength(ledBuffer.getLength());
	}

	/**
	 * Sets the color of the LEDs
	 * 
	 * @param color A Color object reflecting the color you want to use on the LEDs.  i.e.  kRed, kBlue, kSeashell
	 */
  public void setColor(Color color) {
		for (int i = 0; i < 16; i++) {
			ledBuffer.setLED(i, color);
		}
			led.setData(ledBuffer);
			led.start();
	}

	public void setColor(Color color, double tx) {
		MathUtil.clamp(
			tx, 
			-20, 
			20);
		if (tx > -2 && tx < 2) {
			for (int i = 0; i < 16; i++) {
				ledBuffer.setLED(i, color);
			}
			SmartDashboard.putNumber("LED - tx", tx);
		} else if (tx <= -2) {
			double q = 8 - (tx / 2.5);
			int e = (int)q;
			SmartDashboard.putNumber("e", e);

			if (e <= 1) {
				e = 1;
			}
			if (e >= 16){
				e=16;
			}

			for (int i = e; i < 16; i++) {
				ledBuffer.setLED(i, color);
			} 

			for (int i = 0; i < e - 1; i++) {
				ledBuffer.setRGB(i, 0, 0, 0);
			}
		} else if (tx >= 2) {
			double q = 8 - (tx / 2.5);
			int e = (int)q;
			SmartDashboard.putNumber("e2", e);

			if (e <= 1) {
				e = 1;
			}

			for (int i = 0; i < e; i++) {
				ledBuffer.setLED(i, color);
			} 

			for (int i = e + 1; i < 16; i++) {
				ledBuffer.setRGB(i, 0, 0, 0);
			}
		} 
		led.setData(ledBuffer);
		led.start();
	}

  public void setColorLeft(Color color) {
		for (int i = 0; i < 8; i++) {
			ledBuffer.setLED(i, color);
		}
			led.setData(ledBuffer);
			led.start();
	}

  public void setColorRight(Color color) {
		for (int i = 9; i < 16; i++) {
			ledBuffer.setLED(i, color);
		}
			led.setData(ledBuffer);
			led.start();
	}

  public void setColorEdge(Color color) {
		for (int i = 0; i < 2; i++) {
			ledBuffer.setLED(i, color);
		}
    for (int i = 14; i < 16; i++) {
			ledBuffer.setLED(i, color);
		}
			led.setData(ledBuffer);
			led.start();
	}

  public void setColorMiddle(Color color) {
		for (int i = 3; i < 13; i++) {
			ledBuffer.setLED(i, color);
		}
			led.setData(ledBuffer);
			led.start();
	}

	public static void setRed() {
		for (int i = 0; i < 16; i++) {
			ledBuffer.setLED(i, Color.kRed);
		}
			led.setData(ledBuffer);
			led.start();
	}

	public void setGreen() {
		for (int i = 0; i < 16; i++) {
			ledBuffer.setLED(i, Color.kGreen);
		}
			led.setData(ledBuffer);
			led.start();
	}

	public void setBlue() {
		for (int i = 0; i < 16; i++) {
			ledBuffer.setLED(i, Color.kBlue);
		}
			led.setData(ledBuffer);
			led.start();
	}

	public void setYellow() {
		for (int i = 0; i < 16; i++) {
			ledBuffer.setLED(i, Color.kYellow);
		}
			led.setData(ledBuffer);
			led.start();
	}

	public void setSeashell() {
		for (int i = 0; i < 16; i++) {
			ledBuffer.setLED(i, Color.kSeashell);
		}
			led.setData(ledBuffer);
			led.start();
	}

  
	public static void setLeftOff() {
		for (int i = 0; i < 8; i++) {
			ledBuffer.setRGB(i, 0, 0, 0);
		}
			led.setData(ledBuffer);
			led.start();
  }

  
	public static void setRightOff() {
		for (int i = 9; i < 16; i++) {
			ledBuffer.setRGB(i, 0, 0, 0);
		}
			led.setData(ledBuffer);
			led.start();
  }

	public static void setOff() {
		for (int i = 0; i < 16; i++) {
			ledBuffer.setRGB(i, 0, 0, 0);
		}
			led.setData(ledBuffer);
			led.start();
  }
} 