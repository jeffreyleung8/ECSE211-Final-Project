package ca.mcgill.ecse211.navigation;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;
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
	private final static double TILE_SIZE = Main.TILE_SIZE;
	private final static double SENSOR_LENGTH = Main.SENSOR_LENGTH;

	private Odometer odometer;

	private static RobotController robot;

	private static LightSensorController leftLS;
	private static LightSensorController rightLS;

	private double color = 0.25;
	/**
	 * This is a constructor for this class
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param lightSensor
	 */
	public LightLocalizer(Odometer odometer,RobotController robot,LightSensorController leftLS,LightSensorController rightLS ) {
		this.odometer = odometer;
		this.robot = robot;
		this.FORWARD_SPEED = robot.FORWARD_SPEED;
		this.ROTATE_SPEED = robot.ROTATE_SPEED;		
		this.leftLS = leftLS;
		this.rightLS = rightLS;
	}

	/**
	 * This method localizes the robot from the starting point
	 */
	public void initialLocalize() {

		// Start moving the robot forward
		robot.setSpeeds(150,150);
		robot.moveForward();
		
		correct();

		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		robot.setSpeeds(150,150);
		robot.travelDist(-SENSOR_LENGTH);
		robot.turnBy(90,true);
		
		robot.setSpeeds(150, 150);
		robot.moveForward();
		
		correct();
		
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		robot.setSpeeds(150, 150);
		robot.travelDist(-SENSOR_LENGTH);

		robot.turnBy(89, false); //90 is a bit too much
		
		Sound.beep();
		Sound.beep();
		Sound.beep();
	}
	
	/**
	 * This method serves to correct the orientation of the robot for the initial 
	 * light localization
	 */
	private void correct() {
		boolean rightLineDetected = false;
		boolean leftLineDetected = false;
		// Move the robot until one of the sensors detects a line
		while (!leftLineDetected && !rightLineDetected) {
			if(rightLS.fetch() < color || leftLS.fetch() < color) {
				robot.stopMoving();
				if(rightLS.fetch() < color) {
					rightLineDetected = true;
				}
				if(leftLS.fetch() < color) {
					leftLineDetected = true;
				}
			}
		}
		if(!rightLineDetected || !leftLineDetected) {
			if(rightLineDetected) {
				robot.setSpeeds(150, 150);
				robot.startMoving(true, false);
			}
			else if(leftLineDetected){
				robot.setSpeeds(150, 150);
				robot.startMoving(false, true);
			}
		}
		// Keep moving the left/right motor until both lines have been detected
		while ((!leftLineDetected || !rightLineDetected)) {
			// If the other line detected, stop the motors
			if (rightLineDetected && leftLS.fetch() < color) {
				leftLineDetected = true;
				robot.stopMoving();
			} 
			else if (leftLineDetected && rightLS.fetch() < color) {
				rightLineDetected = true;
				robot.stopMoving();
			}
		}
		
	}
		/**
		 * This method localizes the robot at a desired waypoint using the light sensor
		 * @param x - coordinate
		 * @param y - coordinate
		 */
		public void localize(double x, double y) {
			
//	//		//Turn until the robot is not on a black line
//	//		while(lightSensor.fetch() < 20) {
//	//			robot.setSpeeds(100, 100);
//	//			robot.rotate(false);
//	//		}
//			robot.turnBy(45, true);
//			
//			//Start localization
//			int index = 0;
//			double[] lineData = new double[4];
//	
//			robot.setSpeeds(ROTATE_SPEED, ROTATE_SPEED);
//	
//			//Detect the four lines and record the angle at which it is seen
//			while (index < 4) {
//				robot.rotate(true);
//	
//	
//				float sample = lightSensor.fetch();
//	
//				if (sample < 0.30) {
//					lineData[index] = odometer.getXYT()[2];
//					Sound.playNote(Sound.FLUTE, 880, 250);
//					index++;
//				}
//			}
//			robot.stopMoving();
//	
//			double deltax, deltay, thetax, thetay;
//	
//			// calculate our location from 0 using the calculated angles
//			thetay = lineData[3] - lineData[1];
//			thetax = lineData[2] - lineData[4];
//	
//			deltax = -1 * SENSOR_LENGTH * Math.cos(Math.toRadians(thetay / 2));
//			deltay = -1 * SENSOR_LENGTH * Math.cos(Math.toRadians(thetax / 2));
//	
//			// travel to origin to correct position
//			odometer.setXYT(deltax, deltay, odometer.getXYT()[2]);
//			robot.travelTo(0,0);
//	
//			robot.setSpeeds(ROTATE_SPEED/2 , ROTATE_SPEED/2);
//	
//			// if we are not facing 0.0 then turn ourselves so that we are
//			if (odometer.getXYT()[2] <= 350 && odometer.getXYT()[2] >= 10.0) {
//				Sound.beep();
//				robot.turnBy(-odometer.getXYT()[2],true);
//			}
//			
//			robot.stopMoving();
//	
//			odometer.setXYT(x, y, 0.0);
		}
	
		
}
