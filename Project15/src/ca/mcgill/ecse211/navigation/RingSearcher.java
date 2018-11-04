package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.ColorSensorController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * This class implements the ring searcher
 * @author leaakkari
 *
 */
public class RingSearcher {

	ColorSensorController colorSensor;
	private EV3LargeRegulatedMotor leftMotor,rightMotor;
	
	/**
	 *  Constructor for ring searcher
	 * @param colorSensor
	 * @param leftMotor
	 * @param rightMotor
	 */
	public RingSearcher(ColorSensorController colorSensor,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.colorSensor = colorSensor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

	}
	
	/**
	 * This method searches for the orange ring
	 * @param colorOrange
	 * @return true if the ring is found
	 */
	public boolean isFound() {
		return false;
	}
	
}