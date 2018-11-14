package ca.mcgill.ecse211.main;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.enumeration.Team;

/** This class serves to fetch all data from the dpmServer.jar
 * (Beta demo version)
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */

public class WiFi {

	private static final String SERVER_IP = "192.168.2.16";

	//private static final String SERVER_IP = "192.168.2.3";

	private static final int TEAM_NUMBER = 15 ;

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;

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
		//System.out.println("Running..");
		
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
		return Team.GREEN;
	}

	/**
	 * Gets the starting corner of the given team.
	 * 
	 * @return an int representing the starting corner of the team.
	 */
	public int getStartingCorner() {
		return 1;
	}

	/**
	 * Gets the starting corner coordinates of your team.
	 * coords(x,y)
	 * @return starting corner coordinates
	 */
	public int[] getStartingCornerCoords() {
		int[] coords = { 7, 1 };
		return coords;
	}
	/**
	 * Gets the coordinates of the green zone.
	 * 
	 * @return 2-D array containing coordinates of each corner of the green zone
	 */
	public int[][] getGreenZone() {
		// Get coords of green zone
		int llx = ((Long) data.get("Green_LL_x")).intValue();
		int lly = ((Long) data.get("Green_LL_y")).intValue();
		int urx = ((Long) data.get("Green_UR_x")).intValue();
		int ury = ((Long) data.get("Green_UR_y")).intValue();

		// Corner convention:
		// [0] = Lower Left    [1] = Lower Right
		// [2] = Upper Right   [3] = Upper Left
		// [x][y] - 0 <= x <= 3 , 0 <= y <= 1
		int[][] greenZone = { { llx, lly }, { urx, lly }, { urx, ury },{ llx, ury } };

		return greenZone;
	}

	/**
	 * Gets the coordinates of the tunnel
	 * 
	 * @return 2-D array containing coordinates of each corner of the tunnel
	 */
	public int[][] getTunnelZone() {
		int llx, lly, urx, ury;

		// Get coords of red search zone
		llx = ((Long) data.get("TNG_LL_x")).intValue();
		lly = ((Long) data.get("TNG_LL_y")).intValue();
		urx = ((Long) data.get("TNG_UR_x")).intValue();
		ury = ((Long) data.get("TNG_UR_y")).intValue();

		// Corner convention:
		// [0] = Lower Left    [1] = Lower Right
		// [2] = Upper Right   [3] = Upper Left
		// [x][y] - 0 <= x <= 3 , 0 <= y <= 1
		int[][] greenTunnelZone = { { llx, lly }, { urx, lly },{ urx, ury }, { llx, ury } };

		return greenTunnelZone;
	}

	/**
	 * Gets the search zone of the specified team
	 *  
	 * @return 2-D array containing coordinates of each corner of the search zone
	 */
	public int[][] getSearchZone() {
		int llx, lly, urx, ury;
		// Get coords of red search zone
		llx = ((Long) data.get("Island_LL_x")).intValue();
		lly = ((Long) data.get("Island_LL_y")).intValue();
		urx = ((Long) data.get("Island_UR_x")).intValue();
		ury = ((Long) data.get("Island_UR_y")).intValue();

		// Corner convention:
		// [0] = Lower Left    [1] = Lower Right
		// [2] = Upper Right   [3] = Upper Left
		// [x][y] - 0 <= x <= 3 , 0 <= y <= 1

		int[][] greenSearchZone = { { llx, lly }, { urx, lly },{ urx, ury }, { llx, ury } };

		return greenSearchZone;
	}

	/**
	 * Gets the ring set coordinates
	 * 
	 * @return 1-D array of coordinates of the ring set
	 */
	public int[] getRingSet() {
		int x,y;
		//Get coords of ringSet
		x = ((Long) data.get("TG_x")).intValue();
		y = ((Long) data.get("TG_y")).intValue();

		int[] ringSet = {x,y};

		return ringSet;

	}

	/**
	 * Determines the orientation of the tunnel
	 * vertical(along y axis) horizontal(along x-axis)
	 * @return boolean true for vertical, false for horizontal
	 */
	public boolean isTunnelVertical() {

		int llx = ((Long) data.get("TNG_LL_x")).intValue();
		int urx = ((Long) data.get("TNG_UR_x")).intValue();

		if(urx-llx == 1) {
			return true;
		}
		else {
			return false;
		}	
	}
}