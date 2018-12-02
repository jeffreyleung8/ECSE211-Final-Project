package ca.mcgill.ecse211.navigation;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import ca.mcgill.ecse211.main.*;

/** This class serves to drive the robot to the 0 degrees axis.
 * It is using the ultrasonic sensor to read the limit between the wall and the open space.
 * It will read the two limits and calculate an angle to turn which will place
 * the robot parallel to the left wall. This class will be run as a thread as the distance 
 * readings are more accurate and quicker.
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */
public class USLocalizer implements Runnable {
	
	private enum State {RESET,FIRSTWALL,SECONDWALL,CORRECTION,DONE}
	private State state;
	
	// vehicle constants
	public int ROTATE_SPEED;
	private double deltaTheta;

	//Odometer
	private Odometer odometer;

	//Robot
	private RobotController robot;

	//Sensor
	private UltrasonicSensorController usSensor;
	
	
	private boolean isRunning;
	private int filterControl;
	private static final int FILTER_OUT = 30;
	private int prevDistance;
	
	//Constant
	private int OPEN_SPACE = 50;
	private int WALL = 25;
	private int ERROR = 5;
	private double angleA, angleB, turningAngle;

	/**
	 * Constructor of the usLocalizer class
	 * 
	 * @param odometer odometer of the robot (singleton)
	 * @param robot robot controller to control the motors
	 * @param usSensor ultrasonic sensor that is used
	 */
	public USLocalizer(Odometer odometer, RobotController robot,UltrasonicSensorController usSensor) {
		this.odometer = odometer;
		this.robot = robot;
		this.usSensor =usSensor;
		this.ROTATE_SPEED = 150;
		isRunning = true;
		prevDistance = Integer.MAX_VALUE;
		state = State.RESET;
		robot.setSpeeds(175,175);
	}

	/**
	 * Thread run() method which localize the robot
	 * It performs different operations depending the state in which it is in
	 */
	public void run() {
	
		while(isRunning) {
			
			int distance = usSensor.fetch();
			
			if (distance >= 255 && filterControl < FILTER_OUT && prevDistance < distance) {
				filterControl++;
				distance = prevDistance;
			} else if (distance >= 255) {
				// do nothing
			} else {
				filterControl = 0;
			}
			
			switch(state) {
			case RESET:{
				if (distance >= 40) {
					robot.stopMoving();
					//odometer.setTheta(0);
					state = State.FIRSTWALL;
				} 
				//If it is not too far from the wall, start moving counterclockwise until it is
				else {
					robot.rotate(true);
				}
				break;
			}
			case FIRSTWALL:{
				if ((distance <= WALL + ERROR) && (prevDistance > WALL + ERROR))  {
					robot.stopMoving();
					angleA = odometer.getXYT()[2];
					Sound.beep();
					//robot.turnBy(50, true); //turn out of wall
					robot.rotate(true);
					try {
						Thread.sleep(3000);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					state = State.SECONDWALL;
				} 
				else {
					robot.rotate(false);
				}
				break;	
			}
			case SECONDWALL:{
				if ((distance <= WALL + ERROR) && (prevDistance > WALL + ERROR))  {
					robot.stopMoving();
					angleB = odometer.getXYT()[2];
					Sound.beep();
					state = State.CORRECTION;
				} 
				//If no falling edge, keep moving counterclockwise
				else {
					robot.rotate(true);
				}
				break;
			}
			case CORRECTION:{
				
				if (angleA < angleB) {
					deltaTheta = 45 - (angleA + angleB) / 2;

				} else if (angleA > angleB) {
					deltaTheta = 225 - (angleA + angleB) / 2;
				}
				turningAngle = deltaTheta + odometer.getXYT()[2];
				robot.setSpeeds(220, 220);
				robot.turnBy(turningAngle,false);
				odometer.setXYT(0.0, 0.0, 0.0);
				state = State.DONE;
				break;
			}
			case DONE:{ 
				isRunning = false;
				break;
			}
			default: break;
			
			}
			this.prevDistance = distance;
		}
	}

}
