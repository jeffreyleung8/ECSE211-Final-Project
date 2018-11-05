package ca.mcgill.ecse211.tester;

import java.text.DecimalFormat;

import ca.mcgill.ecse211.main.Main;
import lejos.hardware.Sound;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * This class implements testing for the gyro sensor, light sensor, color sensor, ultrasonic sensor and robot turns and navigation
 * @author Jeffrey Leung
 * @author leaakkari
 *
 */
public class Tester {
	
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	
	private EV3ColorSensor lightSensor;
	private SensorMode idColour;
	private float[] colorValue;
	
	
	private EV3ColorSensor colorSensor;
	private SampleProvider rgbValue;
	private float[] rgbData;


	private EV3UltrasonicSensor usSensor;
	private SampleProvider usDistance;
	private float[] usData;
	
	private EV3GyroSensor gyroSensor;
	private SampleProvider gyroAngle;
	private float[] gyroData;
	
	private TextLCD lcd;
	
	/**
	 * This is a constructor for this class
	 * @param leftMotor
	 * @param rightMotor
	 * @param lightSensor
	 * @param colorSensor
	 * @param usSensor
	 * @param gyroSensor
	 * @param lcd
	 */
	public Tester(EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor,EV3ColorSensor lightSensor,EV3ColorSensor colorSensor,
					EV3UltrasonicSensor usSensor,EV3GyroSensor gyroSensor,TextLCD lcd) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		this.lightSensor = lightSensor;
		idColour = this.lightSensor.getRedMode();
		colorValue = new float[idColour.sampleSize()];
		
		this.colorSensor = colorSensor;
		rgbValue = colorSensor.getRGBMode();
		rgbData = new float[rgbValue.sampleSize()];
		
		this.usSensor = usSensor;
		usDistance = usSensor.getMode("Distance");
		usData = new float[usDistance.sampleSize()];

		this.gyroSensor = gyroSensor;
		gyroAngle = gyroSensor.getAngleMode();
		gyroData = new float[gyroAngle.sampleSize()];
		
		this.lcd = lcd;
	}
	
	/**
	 * This method implements a test for the gyro sensor
	 */
	public void testGyro() {
		lcd.clear();
		while (true) {
			gyroAngle.fetchSample(gyroData, 0);
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("Angle: " + numberFormat.format(gyroData[0]), 0, 1);
			Delay.msDelay(500);
			
		}
	}
	/**
	 * This method implements a test for the ultrasonic sensor
	 */
	public void testUS() {
		lcd.clear();
		while(true) {
			usDistance.fetchSample(usData, 0);
			//int distance =  (int) (usData[0] * 100.0);
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("Distance: " + numberFormat.format(usData[0]*100), 0, 1);
			Delay.msDelay(1000);
		}
		
	}
	/**
	 * This method implements a test for the light sensor
	 */
	public void testLS() {
		lcd.clear();
		while(true) {
			idColour.fetchSample(colorValue, 0);
			lcd.drawString("Color intensity: ", 0, 1);
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("  " + numberFormat.format(colorValue[0]), 0, 2);
			Delay.msDelay(1000);
			
		}

	}
	
	/**
	 * This method implements a test for the color sensor
	 */
	public void testCS() {
		lcd.clear();
		while(true) {
			rgbValue.fetchSample(rgbData, 0);
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("R: " + numberFormat.format(rgbData[0]*1000), 0, 2);
			lcd.drawString("G: " + numberFormat.format(rgbData[1]*1000), 0, 3);
			lcd.drawString("B: " + numberFormat.format(rgbData[2]*1000), 0, 4);
			Delay.msDelay(1000);
		}

	}
	
	/**
	 * This method tests the turns of the robot when it is avoiding obstacles
	 */
	public void testTurn() {

		int angle =0;
		for (int i = 1; i < 5; i++) {
			// drive forward two tiles
			leftMotor.setSpeed(200);
			rightMotor.setSpeed(200);

			leftMotor.rotate(convertDistance(Main.WHEEL_RAD, 1 * Main.TILE_SIZE), true);
			rightMotor.rotate(convertDistance(Main.WHEEL_RAD, 1 * Main.TILE_SIZE), false);

			//turn 90 degrees clockwise
			leftMotor.setSpeed(140);
			rightMotor.setSpeed(140);

			leftMotor.rotate(convertAngle(Main.WHEEL_RAD, Main.TRACK, 90.0), true);
			rightMotor.rotate(-convertAngle(Main.WHEEL_RAD, Main.TRACK, 90.0), false);

			angle -= 90;

			leftMotor.stop();
			rightMotor.stop();

			leftMotor.setSpeed(50);
			rightMotor.setSpeed(50);


//			if(gyroSensor.getAngle() > angle) {
//				while(gyroSensor.fetch() >= angle) {
//					leftMotor.forward();
//					rightMotor.backward();
//				}
//			}
//			if(gyroSensor.getAngle() < angle) {
//				while(gyroSensor.fetch() <= angle) {
//					leftMotor.backward();
//					rightMotor.forward(); 
//				}
//			}

			leftMotor.stop();
			rightMotor.stop();
			Sound.beep();

			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

	}
	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return distance
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of a angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @param angle
	 * @return angle
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
