/* This class serves to navigate in the search sepsction and avoids obstacles if detected
 * 
 * @author Jeffrey Leung
 * @author Douglas So
 * @author Lea Akkary
 * @author Yassine Douida
 * @author Tushar Agarwal
 * @author Babette Smith
 */
package ca.mcgill.ecse211.navigation;

import java.text.DecimalFormat;
import java.util.Arrays;

import ca.mcgill.ecse211.colorClassification.ColorClassification;
import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class ObstacleAvoidance extends Thread {

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private LightLocalizer lightSensor;

	// color classification
	private ColorClassification colorClass;

	//fetching data from the sensor
	public float[] usData;
	public SampleProvider usDistance ;

	private TextLCD lcd;

	private double deltax;
	private double deltay;

	// current location of the vehicle
	private double currx;
	private double curry;
	private double currTheta;

	// set constants
	private static final int FORWARD_SPEED = 180;
	private static final int ROTATE_SPEED = 80;
	private static final double TILE_SIZE = 30.48;

	//Variable
	private int llx;
	private int lly;
	private int urx;
	private int ury;
	private int targetRing;

	//Ultrasonic sensor filter variable
	int filterControl = 0;
	int dist = 0;
	private static final int FILTER_OUT = 20;

	private boolean navigate = true;
	public boolean isFinished;

	//maps array : implementing waypoints
	private static double[][]  maps;

	//Counter to pass through the maps[]
	private int i=0;

	// constructor for navigation
	public ObstacleAvoidance(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			SampleProvider us, int[] variable, LightLocalizer lightSensor,EV3ColorSensor colorSensor,TextLCD lcd) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.lightSensor = lightSensor;
		this.isFinished = false;
		colorClass = new ColorClassification(colorSensor);

		this.usDistance = us;
		this.usData = new float[us.sampleSize()];
		llx = variable[0];
		lly = variable[1];
		urx = variable[2];
		ury = variable[3];
		targetRing = variable[4];
		this.lcd = lcd;
	}

	// main run method for navigation
	public void run() {

		constructMap(llx,lly,urx,ury);
		//System.out.println(Arrays.deepToString(maps));
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(300);
		}
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {

		}

		int x = ((urx -llx)*2) + 1; //Size of the array maps[][]
		while(i < x) {
			travelTo(maps[i][0], maps[i][1]);
			//Ring is found, it travels directly to the final point with obstacle avoidance
			if(this.isFinished) {
				travelTo(urx*TILE_SIZE, ury*TILE_SIZE);
				Sound.twoBeeps();
				break;
			}
			else {
				// localize at every 4th point
				if (i % 3 == 0 && i != 0){      
					lightSensor.localize(maps[i][0], maps[i][1]);
					odometer.reset();
				}
				i++;
			}
		}
		//After going to every waypoint, it goes to final point
		travelTo(urx*TILE_SIZE, ury*TILE_SIZE);
		Sound.beepSequenceUp();

	}

	/**
	 * A method to drive our vehicle to a certain Cartesian coordinate
	 * 
	 * @param x
	 *            X-Coordinate
	 * @param y
	 *            Y-Coordinate
	 */

	/**
	 * A method to drive our vehicle to a certain Cartesian coordinate
	 * 
	 * @param x
	 *            X-Coordinate
	 * @param y
	 *            Y-Coordinate
	 */
	private void travelTo(double x, double y) {

		navigate = true;
		determineHeading(x, y);

		// checking obstacles while navigatingh
		while (navigate) {

			int dist = fetchUS();

			// do the adjusted path if reading is lower than 10
			if (dist < 7) {
				lcd.clear();
				/* --------Color Classification-------- */
				leftMotor.stop(true);
				rightMotor.stop();
				// TODO: approach ring, adjust motor speeds

				colorClass.lowerSideMotor();

				float[] rgb;
				int color;
				do{
					rgb = colorClass.fetch();
					color = ColorClassification.findMatch(rgb);
				} while (color == 4); 
				String[] str= {"Blue  ", "Green  ", "Yellow  ", "Orange  ", "None    "};

				DecimalFormat numberFormat = new DecimalFormat("######0.00");
				lcd.drawString("R: " + numberFormat.format(rgb[0]*1000), 0, 3);
				lcd.drawString("G: " + numberFormat.format(rgb[1]*1000), 0, 4);
				lcd.drawString("B: " + numberFormat.format(rgb[2]*1000), 0, 5);
				lcd.drawString("Object Detected ", 0, 6);
				lcd.drawString(str[color], 0, 7);

				if (color  == targetRing - 1){
					Sound.beep();
					this.isFinished = true;				
				} 
				Sound.twoBeeps();
				colorClass.raiseSideMotor();
				rightTurn();
				determineHeading(x, y);
				continue;


			}
			currx = odometer.getXYT()[0];
			curry = odometer.getXYT()[1];

			deltax = x - currx;
			deltay = y - curry;

			// determine our distance from final position
			double hypot = Math.hypot(deltax, deltay);

			// Stop when vehicle is at waypoint
			if (hypot < 0.5) {
				navigate = false;
			}

			// Turn to correct angle towards the endpoint

			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);

			leftMotor.forward();
			rightMotor.forward();

		}
		leftMotor.stop(true);
		rightMotor.stop();
	}




	/**
	 * A method to determine the direction the robot is facing
	 * 
	 * @param x
	 *            X-Coordinate
	 * @param y
	 *            Y-Coordinate
	 */
	private void determineHeading(double x, double y) {
		currx = odometer.getXYT()[0];
		curry = odometer.getXYT()[1];

		deltax = x - currx;
		deltay = y - curry;

		// Calculate the minimum angle to turn around
		currTheta = (odometer.getXYT()[2]) * Math.PI / 180;
		double minTheta = Math.atan2(deltax, deltay) - currTheta;
		turnTo(minTheta);

	}
	/**
	 * A method to turn our vehicle to a certain angle
	 * 
	 * @param theta
	 */
	private void turnTo(double theta) {

		// ensures minimum angle for turning
		if (theta > Math.PI) {
			theta -= 2 * Math.PI;
		} else if (theta < -Math.PI) {
			theta += 2 * Math.PI;
		}

		// set Speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// rotate motors at set speed

		// if angle is negative, turn to the left
		if (theta < 0) {
			leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, -(theta * 180) / Math.PI), true);
			rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, -(theta * 180) / Math.PI), false);

		} else {
			// angle is positive, turn to the right
			leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, (theta * 180) / Math.PI), true);
			rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, (theta * 180) / Math.PI), false);
		}
	}

	/**
	 * This method allows the robot to avoid the obstacle by turning right
	 * and follow a designed path
	 * 
	 */
	public void rightTurn() {

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// turn 90 degrees to avoid obstacle
		leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true);
		rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), false);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		// move to the left by a tile size
		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, TILE_SIZE/2), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, TILE_SIZE/2), false);

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// turn again 90 back and move past the block
		leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true);
		rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), false);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, TILE_SIZE), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, TILE_SIZE), false);

	}
	/**
	 * If one of the motors is moving, the robot is navigating
	 * @return
	 */
	public boolean isNavigating() {
		if((leftMotor.isMoving() || rightMotor.isMoving()))
			return true;
		else 
			return false;
	}
	/**
	 * A method to get the distance from our sensor
	 * 
	 * @return
	 */
	private int fetchUS() {
		usDistance.fetchSample(usData, 0);
		int distance =  (int) (usData[0] * 100.0);
		// rudimentary filter - toss out invalid samples corresponding to null signal 
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value: do not set the distance var, do increment the filter value
			this.filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.dist = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			this.filterControl = 0;
			this.dist = distance;
		}
		if(dist == 2147483647) {
			return 200;
		}

		LCD.drawString(""+ this.dist, 0, 5);
		LCD.clear(5);

		return this.dist;
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
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
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * A method to construct the snake path points 
	 * 
	 * @param llx   x-coordinate of lower-left corner
	 * @param lly   y-coordinate of lower-left corner
	 * @param urx   x-coordinate of upper-right corner
	 * @param ury   y-coordinate of upper-right corner
	 */
	private static void constructMap(int llx, int lly, int urx, int ury) {
		boolean even = false;
		int init = 0;
		int deltax = urx -llx;
		int x = ((urx -llx)*2) + 1;
		maps = new double[x][2];

		//Filling first cell of first column
		maps[0][0]= llx * TILE_SIZE;

		//Filling first column
		int k = llx+1;
		for(int i = 1 ; i < x; i=i+2) {
			maps[i][0]= k * TILE_SIZE;
			maps[i+1][0]= k * TILE_SIZE;
			k++;
		}
		//Filling second column
		int m = ury;
		for(int j = 0; j < x-1; j=j+2) {
			maps[j][1] =  m * TILE_SIZE;
			maps[j+1][1] = m * TILE_SIZE;
			if (m == ury) {
				m = lly;
			}
			else {
				m= ury;
			}
		}
		//Filling last cell of second column
		if(deltax % 2 == 0) {
			maps[x-1][1]=ury * TILE_SIZE;
		}
		else {
			maps[x-1][1]=lly * TILE_SIZE;
		}
	}

}
