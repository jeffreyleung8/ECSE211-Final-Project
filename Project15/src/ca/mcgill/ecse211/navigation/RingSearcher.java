package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.ColorSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import ca.mcgill.ecse211.enumeration.SearchState;
import ca.mcgill.ecse211.main.Main;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import java.util.ArrayList;

/**
 * This class implements the ring searcher using the color sensor
 * It is implemented as a thread that collect one ring
 *
 * @author Jeffrey Leung
 * @author leaakkari
 *
 */
public class RingSearcher implements Runnable {

	private enum State {APPROACHING,DETECTING,GRABING};
	private State state;
	//Robot
	private RobotController robot;

	//Sensors
	private ColorSensorController colorSensor;

	public static final double SENSOR_LENGTH = 3.3;

	//Odometer
	private Odometer odometer;

	//Search state
	public SearchState searchState;

	//Constants
	private long START_TIME = Main.START_TIME;

	//Odometry correction
	private OdometryCorrection odoCorr;

	//Side motors
	private static final EV3LargeRegulatedMotor leftSideMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	private static final EV3LargeRegulatedMotor rightSideMotor = 
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

	/**
	 *  Constructor for ring searcher
	 * @param colorSensor color sensor that is used
	 * @param odometer odometer of the robot (singleton)
	 * @param usSensor usSensor that is used
	 * @param robot robot controller to control the motors
	 */
	public RingSearcher(Odometer odometer,ColorSensorController colorSensor, RobotController robot) {
		this.colorSensor = colorSensor;
		this.robot = robot;
		this.odometer = odometer;
		searchState = SearchState.IN_PROGRESS;
		state = State.APPROACHING;

	}

	/**
	 * Thread run() method which search for the closest ring
	 * It performs different operations depending the state in which it is in
	 * Search is ended if time is passed 4 minutes or the ring is grabbed
	 */
	@Override 
	public void run(){
		while(searchState == SearchState.IN_PROGRESS) {

			long timeElapsed = System.currentTimeMillis() - START_TIME;
			//Time out at 4 min
			if(timeElapsed >= 240000) {
				searchState = SearchState.TIME_OUT;
			}

			float[] rgb = colorSensor.fetch();

			switch(state) {
			case APPROACHING:{
				odoCorr.correct(odometer.getXYT()[2]);
				robot.setSpeeds(100, 100);
				robot.travelDist(8.2,100); //Distance to reach the ring

				state = State.DETECTING;
				break;
			}
			case DETECTING:{
				int color = 0;
				while(color == 0) {
					ArrayList<Integer> samples = new ArrayList<Integer>(300);
					//100 samples
					for(int i = 0 ; i < 300; i++){
						color = colorSensor.findMatch(rgb);
						samples.add(i,color);
					}
					int[] data = new int[5];

					for(Integer sample : samples){
						data[sample]++;
					}
					int max = data[0];
					color = 0;
					for(int j = 1; j < 5; j++){
						if(data[j] > max){
							max = data[j];
							color = j;
						}
					}
				}
				if(color != 0) {
					colorSensor.setTargetColor(4);
					colorSensor.beep();
					state = State.GRABING;
				}
				break;
			}
			case GRABING:{
				robot.setSpeeds(80, 80);
				robot.travelDist(8.5,150); //Distane to grab the ring
				robot.travelDist(-13,150); //Distance to back off
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(SENSOR_LENGTH,150);
				searchState = SearchState.RING_FOUND;
				break;
			}
			}
		}
	}


	/**
	 * Sets the OdometryCorrection object to be used by the robot controller.
	 * 
	 * @param odoCorrection the OdometryCorrection object to be used
	 */
	public void setOdoCorrection(OdometryCorrection odoCorrection) {
		this.odoCorr = odoCorrection;
	}
}





