
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
	public static int leftOuterIntake = 13;
	public static int rightOuterIntake = 14;



	////////// These are all of the solenoids for the robot
	
	//Shifting

	//Squid
	public static int squidOpenPort = 0;
	public static int squidClosePort = 1;
	public static int squidExtendForward = 2;
	public static int squidExtendBackward = 3;

	//BeaverTail
	public static int beaverTailEjectExtend = 6;
	public static int beaverTailEjectRetract = 7;

	//CargoIntakeSubsystem
	public static int cargoIntakeExtend = 4;
	public static int cargoIntakeRetract = 5;



	///// DIOs

	public static int liftBottomLimit = 0;
	public static int liftHighLimit = 1;
	public static int rearLeftTrigger = 6;
	public static int rearLeftResponse = 7;
	public static int rearRightTrigger = 8;
	public static int rearRightResponse = 9;
	public static int leftTrigger = 10;
	public static int leftResponse = 11;
	public static int rightTrigger = 12;
	public static int rightResponse = 2;
	public static int hatchDetector = 3;

	///////////// PDP PORTS



}
