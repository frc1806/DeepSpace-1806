
package org.usfirst.frc.team1806.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    ///////////// CAN ports 
	
	//Drive train CAN
	public static int rightA = 0; //
	public static int masterRight = 1; //
	public static int rightC = 2;
	public static int leftA = 3;  //
	public static int masterLeft = 4;  //
	public static int leftC = 5; //

	
	//Cube Elevator CANs
	public static int liftLead = 6;
	public static int liftFollow = 7;

	
	//Intake CAN Ports
	
	public static int leftInnerIntake = 11;
	public static int rightInnerIntake = 12;


	////////// These are all of the solenoids for the robot
	
	//Shifting

	//Squid
	public static int squidopenport  = 0;
	public static int squidcloseport = 1;

	///// DIOs

	public static int liftBottomLimit = 0;
	public static int liftHighLimit = 1;


	///////////// PDP PORTS



}
