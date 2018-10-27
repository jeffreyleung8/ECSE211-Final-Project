/* This class serves to detect and classify rings colors
 * 
 * @author Jeffrey Leung
 * @author Douglas So
 * @author Lea Akkary
 * @author Yassine Douida
 * @author Tushar Agarwal
 * @author Babette Smith
 */
package ca.mcgill.ecse211.colorClassification;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class ColorClassification {

	// Motor objects
	private static final EV3LargeRegulatedMotor sideMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
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


	//Constructor
	public ColorClassification(EV3ColorSensor colorSensor) {
		this.colorSensor = colorSensor;
		rgbValue = colorSensor.getMode("RGB");
		rgbData = new float[rgbValue.sampleSize()];
	}

	/**
	 * This method allows to lower the light sensor to an
	 * appropriate level to detect the ring
	 * 
	 */
	public void lowerSideMotor() {
		sideMotor.setSpeed(60);
		sideMotor.rotate(93);
	}

	/**
	 * This method allows to bring back the light sensor 
	 * to its initial condition
	 * 
	 */
	public void raiseSideMotor() {
		sideMotor.setSpeed(60);
		sideMotor.rotate(-93);
	}

	/**
	 * This method allows to detect the color of the ring
	 * @return (integer representing the color)
	 */
	public int detect() {
		int color;
		do {
			color = findMatch( fetch() );
			//   System.out.println("stuck in detect()");
		} while (color == 4);
		return color;
	}

	// Return RGB values to rgbData
	/**
	 * This method allows to collect rgb values
	 * @return (array containing rgb values) 
	 */

	public float[] fetch() {
		rgbValue.fetchSample(rgbData, 0);
		return rgbData;
	}

	/**
	 * This method allows to match the readings and the mean to 
	 * determine the color detected
	 * @return (integer representing color)
	 */
	public static int findMatch(float array[]) {

		float euc=(float)Math.sqrt((Math.pow(array[0], 2)+Math.pow(array[1], 2)+Math.pow(array[2], 2)));

		// normalize
		float R=array[0]/euc;
		float G=array[1]/euc;
		float B=array[2]/euc;

		// 
		for (int i=0; i<4; i++) {
			float differenceR=Math.abs(R-(float)mean[i][0])/0.05f;
			float differenceG=Math.abs(G-(float)mean[i][1])/0.05f;
			float differenceB=Math.abs(B-(float)mean[i][2])/0.05f;
			if (differenceR<1.0  &&differenceG<1.0 && differenceB<1.0) {
				return i;
			}
		}

		return 4;
	}
}