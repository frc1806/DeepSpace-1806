
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

	public static int masterRight = 1; //
	public static int rightC = 2;
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
	public static int cargoIntakeExtend = 6;
	public static int cargoIntakeRetract = 7;



	////////// These are all of the solenoids for the robotri
	
	//Shifting

	//Squid

	public static int squidOpenPort = 1;
	public static int squidClosePort = 0; //1
	public static int squidExtendForward = 3;
	public static int squidExtendBackward = 2;

	//BeaverTail

	//public static int beaverTailEjectExtend = 6;
	//public static int beaverTailEjectRetract = 7;

	//LiftStand
	public static int liftStandUp = 6;
	public static int liftLeanBack = 7;




	///// DIOs

	public static int liftBottomLimit = 0;
	public static int liftHighLimit = 1;
	public static int rearLeftTrigger = 4;
	public static int rearLeftResponse = 5;
	public static int rearRightTrigger = 8;
	public static int rearRightResponse = 9;
	public static int leftTrigger = 10;
	public static int leftResponse = 11;
	public static int rightTrigger = 12;
	public static int rightResponse = 2;
	public static int hatchDetector = 3;

	//HABClimber

	public static int HABLiftLeft = 15;
	public static int HABLiftRight = 16;

	///////////// PDP PORTS



}
