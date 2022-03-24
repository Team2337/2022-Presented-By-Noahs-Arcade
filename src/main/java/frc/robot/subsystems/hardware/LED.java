package frc.robot.subsystems.hardware;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.LED.LEDOff;

/**
 * Where all the LEDs are controlled
 * 
 * @author Zayd A.
 */

public class LED extends SubsystemBase {
	/*
	 * PREDEFINED COLORS - Add more as needed
	 * http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf#page=14
	 */
	public static double green = 0.75;
	public static double off = 0.99;// When the robot is enabled
	public static double rainbow = -0.99; // When the robot is disabled
	public static double red = 0.61;
	public static double strobeWhite = -0.05; // When the cargo intake is running
	public static double white = 0.92;
	public static double blue = 0.97;
	public static double purple = 0.96;
	public static double pink = 0.88;
	public static double darkBlue = 0.84;
	public static double aqua = 0.83;
	public static double lightBlue = 0.7;
	public static double yellow = 0.65;

	// The Blinkin LED controller is treated as a Spark
	private static AddressableLED led;
	private static AddressableLEDBuffer ledBuffer;

	// The currently set color
	private static double setColor = 0;

	/**
	 * Controls the LEDs on the Robot using a Blinkin
	 * 
	 * @param pwm The PWM port that the blinkin is plugged into
	 */
	public LED() {
		led = new AddressableLED(Constants.BLINKIN_PWM_ID);
		ledBuffer = new AddressableLEDBuffer(16);
		led.setLength(ledBuffer.getLength());
	}

	/**
	 * Sets the color of the LEDs
	 * 
	 * @param color A decimal value from -1 to 1 representing a preset color
	 */
	public static void setRed() {
		for (int i = 0; i < 16; i++) {
			ledBuffer.setLED(i, Color.kRed);
		}
			led.setData(ledBuffer);
			led.start();
	}

	public static void setGreen() {
		for (int i = 0; i < 16; i++) {
			ledBuffer.setLED(i, Color.kGreen);
		}
			led.setData(ledBuffer);
			led.start();
	}

	public static void setBlue() {
		for (int i = 0; i < 16; i++) {
			ledBuffer.setLED(i, Color.kBlue);
		}
			led.setData(ledBuffer);
			led.start();
	}

	public static void setYellow() {
		for (int i = 0; i < 16; i++) {
			ledBuffer.setLED(i, Color.kYellow);
		}
			led.setData(ledBuffer);
			led.start();
	}

	public static void setSeashell() {
		for (int i = 0; i < 16; i++) {
			ledBuffer.setLED(i, Color.kSeashell);
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