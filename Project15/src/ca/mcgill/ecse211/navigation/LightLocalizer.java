package ca.mcgill.ecse211.navigation;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.main.*;

/** This class serves to drive the cart to the origin
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */
public class LightLocalizer {

	//Constants
	private final int FORWARD_SPEED;
	private final int ROTATE_SPEED;
	private final double TILE_SIZE;
	private final double SENSOR_LENGTH;
	

	private Odometer odometer;

	private RobotController robot;
	
	private LightSensorController lightSensor;


	double[] lineData;

	/**
	 * This is a constructor for this class
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param lightSensor
	 */
	public LightLocalizer(Odometer odometer,RobotController robot,LightSensorController lightSensor) {
		this.odometer = odometer;
		this.robot = robot;
		this.FORWARD_SPEED = robot.FORWARD_SPEED;
		this.ROTATE_SPEED = robot.ROTATE_SPEED;
		this.TILE_SIZE = Main.TILE_SIZE;
		this.SENSOR_LENGTH = Main.SENSOR_LENGTH;
		
		this.lightSensor = lightSensor;
		lineData = new double[4];
	}
	
	/**
	 * This method localizes the robot from the starting point
	 * @param x - coordinate
	 * @param y - coordinate
	 */
	public void initialLocalize(double x, double y) {
		
		moveToOrigin();
		
		int index = 0;
		robot.setSpeeds(ROTATE_SPEED, ROTATE_SPEED);

		//Detect the four lines and record the angle at which it is seen
		while (index < 4) {
			robot.rotate(true);

			float sample = lightSensor.fetch();

			if (sample < 0.30) {
				lineData[index] = odometer.getXYT()[2];
				Sound.playNote(Sound.FLUTE, 880, 250);
				index++;
			}
		}
		robot.stopMoving();

		double deltax, deltay, thetax, thetay;

		// calculate our location from 0 using the calculated angles
		thetay = lineData[3] - lineData[1];
		thetax = lineData[2] - lineData[0];

		deltax = -1 * SENSOR_LENGTH * Math.cos(Math.toRadians(thetay / 2));
		deltay = -1 * SENSOR_LENGTH * Math.cos(Math.toRadians(thetax / 2));

		// travel to origin to correct position
		odometer.setXYT(deltax, deltay, odometer.getXYT()[2]);
		robot.travelTo(0,0);
		
		robot.setSpeeds(ROTATE_SPEED/2 , ROTATE_SPEED/2);

		// if we are not facing 0.0 then turn ourselves so that we are
		if (odometer.getXYT()[2] <= 350 && odometer.getXYT()[2] >= 10.0) {
			robot.turnBy(-odometer.getXYT()[2],true);
			Sound.beep();
		}
		
		robot.stopMoving();
		odometer.setXYT(x, y, 0.0);
	}

	/**
	 * This method localizes the robot using the light sensor to precisely move to
	 * the right location
	 * @param x - coordinate
	 * @param y - coordinate
	 */
	public void localize(double x, double y) {
		
		//Turn until the robot is not on a black line
		while(lightSensor.fetch() < 20) {
			robot.setSpeeds(100, 100);
			robot.rotate(false);
		}
		
		//Start localization
		int index = 0;
		robot.setSpeeds(ROTATE_SPEED, ROTATE_SPEED);

		//Detect the four lines and record the angle at which it is seen
		while (index < 4) {
			robot.rotate(true);


			float sample = lightSensor.fetch();

			if (sample < 0.30) {
				lineData[index] = odometer.getXYT()[2];
				Sound.playNote(Sound.FLUTE, 880, 250);
				index++;
			}
		}
		robot.stopMoving();

		double deltax, deltay, thetax, thetay;

		// calculate our location from 0 using the calculated angles
		thetay = lineData[3] - lineData[1];
		thetax = lineData[2] - lineData[4];

		deltax = -1 * SENSOR_LENGTH * Math.cos(Math.toRadians(thetay / 2));
		deltay = -1 * SENSOR_LENGTH * Math.cos(Math.toRadians(thetax / 2));

		// travel to origin to correct position
		odometer.setXYT(deltax, deltay, odometer.getXYT()[2]);
		robot.travelTo(0,0);

		robot.setSpeeds(ROTATE_SPEED/2 , ROTATE_SPEED/2);

		// if we are not facing 0.0 then turn ourselves so that we are
		if (odometer.getXYT()[2] <= 350 && odometer.getXYT()[2] >= 10.0) {
			Sound.beep();
			robot.turnBy(-odometer.getXYT()[2],true);
		}
		
		robot.stopMoving();

		odometer.setXYT(x, y, 0.0);
	}

	/**
	 * This method moves the robot towards the origin
	 */
	public void moveToOrigin() {

		robot.turnTo(Math.PI /4);
		
//		if(odometer.getXYT()[2] <= 10.00 || odometer.getXYT()[2] >=350) {
//			robot.turnTo(Math.PI /4);
//		}
		
		robot.setSpeeds(ROTATE_SPEED+60, ROTATE_SPEED+60);


		// get sample
		float sample = lightSensor.fetch();

		// move forward past the origin until light sensor sees the line
		while (sample > 0.30) {

			sample = lightSensor.fetch();
			robot.moveForward();

		}
		robot.stopMoving();
		Sound.beep();

		// Move backwards so our origin is close to origin
		robot.travelDist(-12.5);

	}
}
