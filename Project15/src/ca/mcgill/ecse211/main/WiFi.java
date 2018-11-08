package ca.mcgill.ecse211.main;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.enumeration.Team;

/** This class serves to fetch all data from the server JAR
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */

public class WiFi {


	private static final String SERVER_IP = "192.168.2.3";

	private static final int TEAM_NUMBER = 15 ;

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;

	// Create Map variable
	private Map data;

	@SuppressWarnings("rawtypes")
	public WiFi() {
		// Store the data
		this.getData();

		// Clear the console
		System.out.flush();
	}

	/**
	 * Stores all the data sent by the server JAR file into a Map object.
	 */
	public void getData() {
		System.out.println("Running..");
		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data, catching any errors that might
		// occur
		try {
			/*
			 * getData() will connect to the server and wait until the user/TA
			 * presses the "Start" button in the GUI on their laptop with the
			 * data filled in. Once it's waiting, you can kill it by pressing
			 * the upper left hand corner button (back/escape) on the EV3.
			 * getData() will throw exceptions if it can't connect to the server
			 * (e.g. wrong IP address, server not running on laptop, not
			 * connected to WiFi router, etc.). It will also throw an exception
			 * if it connects but receives corrupted data or a message from the
			 * server saying something went wrong. For example, if TEAM_NUMBER
			 * is set to 1 above but the server expects teams 17 and 5, this
			 * robot will receive a message saying an invalid team number was
			 * specified and getData() will throw an exception letting you know.
			 */
			data = conn.getData();

		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}
	}

	/**
	 * Gets the color of the team you are in.
	 * 
	 * @return Team color enumeration for the team you are in
	 */
	public Team getTeam() {
		// Return the corresponding team colour for the team number (8)
		if (((Long) data.get("RedTeam")).intValue() == TEAM_NUMBER) {
			return Team.RED;
		} else if (((Long) data.get("GreenTeam")).intValue() == TEAM_NUMBER) {
			return Team.GREEN;
		} else {
			return null;
		}
	}

	/**
	 * Gets the starting corner of the given team.
	 * 
	 * @return an int representing the starting corner of the team.
	 */
	public int getStartingCorner(Team team) {
		// Check which team we are
		if (team == Team.RED) {
			// Return the corresponding starting corner for our team
			return ((Long) data.get("RedCorner")).intValue();
		} else if (team == Team.GREEN) {
			// Repeat for green team
			return ((Long) data.get("GreenCorner")).intValue();
		}
		return -1;
	}

	/**
	 * Gets the starting corner coordinates of your team.
	 * coords(x,y)
	 * @return starting corner coordinates
	 */
	public int[] getStartingCornerCoords() {
		int[] coords = { 0, 0 };
		switch (getStartingCorner(this.getTeam())) {
		case 0:
			coords[0] = 1;
			coords[1] = 1;
			break;
		case 1:

			coords[0] = 14;
			coords[1] = 1;
			break;
		case 2:
			coords[0] = 14;
			coords[1] = 8;
			break;
		case 3:
			coords[0] = 1;
			coords[1] = 8;
			break;
		}
		return coords;
	}

	/**
	 * Gets the coordinates of the red zone.
	 * 
	 * @return two dimensional int array containing the four corners of the red zone as (x, y) pairs
	 */
	public int[][] getRedZone() {
		// Get coords of red zone
		int lowerLeftX = ((Long) data.get("Red_LL_x")).intValue();
		int lowerLeftY = ((Long) data.get("Red_LL_y")).intValue();
		int upperRightX = ((Long) data.get("Red_UR_x")).intValue();
		int upperRightY = ((Long) data.get("Red_UR_y")).intValue();

		// Corner convention:
		// [0] = Lower Left
		// [1] = Lower Right
		// [2] = Upper Right
		// [3] = Upper Left
		int[][] redZone = { { lowerLeftX, lowerLeftY }, { upperRightX, lowerLeftY }, { upperRightX, upperRightY },
				{ lowerLeftX, upperRightY } };

		return redZone;
	}

	/**
	 * Gets the coordinates of the green zone.
	 * 
	 * @return two dimensional int array containing the four corners of the green zone as (x, y) pairs
	 */
	public int[][] getGreenZone() {
		// Get coords of green zone
		int llx = ((Long) data.get("Green_LL_x")).intValue();
		int lly = ((Long) data.get("Green_LL_y")).intValue();
		int urx = ((Long) data.get("Green_UR_x")).intValue();
		int ury = ((Long) data.get("Green_UR_y")).intValue();

		// Corner convention:
		// [0] = Lower Left
		// [1] = Lower Right
		// [2] = Upper Right
		// [3] = Upper Left
		int[][] greenZone = { { llx, lly }, { urx, lly }, { urx, ury },{ llx, ury } };

		return greenZone;
	}

	/**
	 * Gets the coordinates of the tunnel.
	 * 
	 * @return two dimensional int array containing the four corners of the tunnel as (x, y) pairs
	 */
	public int[][] getTunnelZone(Team team) {
		int llx, lly, urx, ury;
		switch (team) {
		case RED:
			// Get coords of red search zone
			llx = ((Long) data.get("BRR_LL_x")).intValue();
			lly = ((Long) data.get("BRR_LL_y")).intValue();
			urx = ((Long) data.get("BRR_UR_x")).intValue();
			ury = ((Long) data.get("BRR_UR_y")).intValue();

			// Corner convention:
			// [0] = Lower Left
			// [1] = Lower Right
			// [2] = Upper Right
			// [3] = Upper Left
			int[][] redTunnelZone = { { llx, lly }, { urx, lly },{ urx, ury }, { llx, ury } };

			return redTunnelZone;
			
		case GREEN:
			// Get coords of red search zone
			llx = ((Long) data.get("BRG_LL_x")).intValue();
			lly = ((Long) data.get("BRG_LL_y")).intValue();
			urx = ((Long) data.get("BRG_UR_x")).intValue();
			ury = ((Long) data.get("BRG_UR_y")).intValue();

			// Corner convention:
			// [0] = Lower Left
			// [1] = Lower Right
			// [2] = Upper Right
			// [3] = Upper Left
			int[][] greenTunnelZone = { { llx, lly }, { urx, lly },{ urx, ury }, { llx, ury } };

			return greenTunnelZone;
		default:
			return null;
		}
	}

	/**
	 * Gets the search zone of the specified team
	 * 
	 * @param team the team of the search zone wanted
	 * @return two dimensional int array containing the four corners of the search zone as (x, y) pairs
	 */
	public int[][] getSearchZone(Team team) {
		int llx, lly, urx, ury;

		switch (team) {
		case RED:
			// Get coords of red search zone
			llx = ((Long) data.get("TR_LL_x")).intValue();
			lly = ((Long) data.get("TR_LL_y")).intValue();
			urx = ((Long) data.get("TR_UR_x")).intValue();
			ury = ((Long) data.get("TR_UR_y")).intValue();

			// Corner convention:
			// [0] = Lower Left
			// [1] = Lower Right
			// [2] = Upper Right
			// [3] = Upper Left
			int[][] redSearchZone = { { llx, lly }, { urx, lly },{ urx, ury }, { llx, ury } };

			return redSearchZone;
			
		case GREEN:
			// Get coords of red search zone
			llx = ((Long) data.get("TG_LL_x")).intValue();
			lly = ((Long) data.get("TG_LL_y")).intValue();
			urx = ((Long) data.get("TG_UR_x")).intValue();
			ury = ((Long) data.get("TG_UR_y")).intValue();

			// Corner convention:
			// [0] = Lower Left
			// [1] = Lower Right
			// [2] = Upper Right
			// [3] = Upper Left
			int[][] greenSearchZone = { { llx, lly }, { urx, lly },{ urx, ury }, { llx, ury } };

			return greenSearchZone;
		default:
			return null;
		}
	}

}