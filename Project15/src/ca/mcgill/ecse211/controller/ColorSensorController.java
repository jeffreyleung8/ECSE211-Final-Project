package ca.mcgill.ecse211.controller;

import java.text.DecimalFormat;
import java.util.Arrays;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

/** This class serves to detect and classify rings colors
 * It is used in the ringSearcher class in the ring searching process
 * It can detect the color blue, green, yellow and orange
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */
public class ColorSensorController {

	private EV3ColorSensor colorSensor;
	// Light sensor objects
	private SampleProvider rgbValue;
	// Float arrays for Color data
	private float[] rgbData;
	
	// RGB Mean Values
	private static final float[][] mean= {
			{0.170390332f,0.767597595f,0.617868163f},
			{0.402231815f,0.906190081f,0.13049561f},
			{0.832694447f,0.538629888f,0.128443766f},
			{0.953786617f,0.290982684f,0.074967764f}};
	
	//Target color
	private int targetColor = 0;
	
	/**
	 * This is a constructor for the ColorSensorController class
	 * @param colorSensor the color sensor to use
	 */
	public ColorSensorController(EV3ColorSensor colorSensor) {
		this.colorSensor = colorSensor;
		rgbValue = colorSensor.getRGBMode();
		rgbData = new float[rgbValue.sampleSize()];
	}

	
	/**
	 * This method allows to collect rgb values detected by the color sensor
	 * @return array containing rgb values read by color sensor
	 */

	public float[] fetch() {
		rgbValue.fetchSample(rgbData, 0);
		return rgbData;
	}
	
	/** 
	 *This method allows to beep n times depending the color
	 * 1-blue 2-green 3-yellow 4-orange 0-None 
	 * @return integer representing the color
	 */

	public void beep() {
		switch(this.targetColor) {
		case 1: {
			Sound.beep();
			LCD.drawString("blue", 0, 5);
			break;
		}
		case 2:{
			Sound.twoBeeps();
			LCD.drawString("green", 0, 5);
			break;
		}
		case 3:{
			Sound.beep();
			Sound.twoBeeps();
			LCD.drawString("yellow", 0, 5);
			break;
		}
		case 4:{
			Sound.twoBeeps();
			Sound.twoBeeps();
			LCD.drawString("orange", 0, 5);
			break;
		}
		default: break;
		}
	}
	
	/**
	 * This method allows to set the target color
	 * 1-blue 2-green 3-yellow 4-orange 0-None 
	 * @param color integer representing color
	 */
	public void setTargetColor(int color){
		this.targetColor = color;
	}
	
	/**
	 * This method allows to match the readings and the mean to determine the color detected
	 * 1-blue 2-green 3-yellow 4-orange 0-None 
	 * @param array array containing rgb values
	 * @return integer representing color
	 */
	
	public int findMatch(float array[]) {
		
		double blue,green,yellow,orange,none;
		

		double euc = (Math.sqrt(array[0]*array[0] + array[1]*array[1] +array[2]*array[2]));
		
		// normalize
		double R=array[0]/euc;
		double G=array[1]/euc;
		double B=array[2]/euc;
		
		blue = Math.sqrt(Math.pow(R - mean[0][0], 2) + Math.pow(G - mean[0][1], 2) + Math.pow(B - mean[0][2], 2));
		green = Math.sqrt(Math.pow(R - mean[1][0], 2) + Math.pow(G - mean[1][1], 2) + Math.pow(B - mean[1][2], 2));
		yellow = Math.sqrt(Math.pow(R - mean[2][0], 2) + Math.pow(G - mean[2][1], 2) + Math.pow(B - mean[2][2], 2));
		orange = Math.sqrt(Math.pow(R - mean[3][0], 2) + Math.pow(G - mean[3][1], 2) + Math.pow(B - mean[3][2], 2));
		none = Math.sqrt(Math.pow(R - mean[4][0], 2) + Math.pow(G - mean[4][1], 2) + Math.pow(B - mean[4][2], 2));
		double[] list = {blue, green, yellow, orange, none};

		//sorted array
		Arrays.sort(list);
		
		if(list[0]== blue) {
			targetColor = 1;
			return 1;
		}
		else if(list[0]== green) {
			targetColor = 2;
			return 2;
		}
		else if(list[0]== yellow) {
			targetColor = 3;
			return 3;
		}
		else if(list[0]== orange) {
			targetColor = 4;
			return 4;
		}
		else {
			return 0;
		}
		
	}
}