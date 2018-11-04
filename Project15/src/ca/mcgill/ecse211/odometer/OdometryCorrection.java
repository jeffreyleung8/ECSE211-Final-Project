
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
/**
 * This class is implemented for odometer correction
 * @author leaakkari
 *
 */
public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private static final double TILE_LENGTH = 30.48;
	private static int nbXLines; // counter for black line in x-axis
	private static int nbYLines; // counter for black line in y-axis
	private Odometer odometer;
	private float[] lsData;
	private static Port lsPort = LocalEV3.get().getPort("S1");
	private SampleProvider color;
	private SensorModes lightSensor;
	private double correctionInX;
	private double correctionInY;

	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.  
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();
		this.lightSensor = new EV3ColorSensor(lsPort);
		this.color = lightSensor.getMode("Red");
		this.lsData = new float[lightSensor.sampleSize()];
		nbXLines = 0; 
		nbYLines = 0;
		correctionInX = 0.0;
		correctionInY = 0.0;
	}

	/**
	 * Here is where the odometer correction code should be run.  
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		float colorIntensity;

		while (true) {

			correctionStart = System.currentTimeMillis();

			// TODO Trigger correction (When do I have information to correct?)
			// TODO Calculate new (accurate) robot position
			// TODO Update odometer with new calculated (and more accurate) values

			color.fetchSample(lsData, 0);
			colorIntensity = lsData[0];

			double[] odoData = odometer.getXYT(); // using get method from OdometerData
			double theta = odoData[2];

			if (colorIntensity < 0.25) { // black lines reflect low intensity light
				Sound.playNote(Sound.FLUTE, 880, 250);
				
				if (theta < 20 || theta > 340) { // theta around 0 degrees; moving in positive y direction
					odometer.setY(nbYLines * TILE_LENGTH - correctionInY); // calculating and updating odometer with more
					// accurate values
					nbYLines ++; 
				}
				if (theta < 110 && theta >= 70) { // theta around 90degrees; moving in positive x direction
					odometer.setX(nbXLines * TILE_LENGTH - correctionInX);
					nbXLines ++;
				}

				if (theta < 200 && theta >= 160) { // theta around 180 degrees; moving in negative y direction
					odometer.setY((nbYLines - 1) * TILE_LENGTH + correctionInY);
					nbYLines = nbYLines - 1; 

				}

				if (theta < 285 && theta >= 255) { // theta around 270 degrees; moving in negative x direction
					odometer.setX((nbXLines - 1) * TILE_LENGTH + correctionInX);
					nbXLines = nbXLines - 1; 
				}

				// this ensure the odometry correction occurs only once every period
				correctionEnd = System.currentTimeMillis();
				if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
					try {
						Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
					} catch (InterruptedException e) {
						// there is nothing to be done here
					}
				}
			}
		}
	}
}

