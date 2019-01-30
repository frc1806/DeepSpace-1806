/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1806.robot;

import edu.wpi.first.wpilibj.CircularBuffer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team1806.robot.auto.actions.controller.VibrateControllerForTime;
import org.usfirst.frc.team1806.robot.subsystems.*;
import org.usfirst.frc.team1806.robot.util.CheesyDriveHelper;
import org.usfirst.frc.team1806.robot.util.Latch;
import org.usfirst.frc.team1806.robot.util.XboxController;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	private DriveTrainSubsystem mDriveTrainSubsystem = DriveTrainSubsystem.getInstance();
	private SquidSubsystem mSquidSubsystem = SquidSubsystem.getInstance();
	private CompressorControlSubsystem mCompressorControlSubsystem = CompressorControlSubsystem.getInstance();
	private LiftSubsystem mLiftSubsystem = LiftSubsystem.getInstance();
	private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
	private XboxController dc = new XboxController(0);
	private XboxController oc = new XboxController(1);
	private XboxController autoController = new XboxController(2);
	private Latch autoInTeleOp = new Latch();
	private Boolean wasB = false;
	private Boolean wasRightDPad = false;
	private CargoIntakeSubsystem mCargoIntakeSubsystem = CargoIntakeSubsystem.getInstance();

	public void runCommands(){
		synchronized (mDriveTrainSubsystem) {
			if(dc.getRightTrigger() > .2) {
				mDriveTrainSubsystem.setCreepMode(mCheesyDriveHelper.cheesyDrive(
						dc.getLeftJoyY(), dc.getRightJoyX(), dc.getButtonRB() , mDriveTrainSubsystem.isHighGear()));
			}else {
				mDriveTrainSubsystem.setOpenLoop(mCheesyDriveHelper.cheesyDrive(
						dc.getLeftJoyY(), dc.getRightJoyX(), dc.getButtonRB() , mDriveTrainSubsystem.isHighGear()));
			}
		}
		synchronized (mLiftSubsystem) {
			if(dc.getButtonA()) {
				mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.TELEOP_HOLD);
			}
			if(dc.getButtonX()) {
				mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.SHIP_CARGO);
			}
			if(dc.getButtonY()) {
				mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.ROCKET_CARGO_HIGH);
			}
		}

		if(Constants.enableAutoInTeleOp){
			autoInTeleOp.update(autoController.getButtonStart());
		}

		if(!wasB && dc.getButtonB()){
			if(mSquidSubsystem.isOpen()) {
				mSquidSubsystem.closeSquid();
			}
			else {
				mSquidSubsystem.openSquid();
			}
		}

		if(!wasRightDPad && dc.getPOVRight()) {
			if(mSquidSubsystem.isExtended()) {
				mSquidSubsystem.retractSquid();
			}
			else {
				mSquidSubsystem.isExtended();
			}
		}
		if(dc.getLeftTrigger() >.2){
			mCargoIntakeSubsystem.intakeCargo();
		}
		else if(dc.getPOVDown()){
			mCargoIntakeSubsystem.scoreCargo(CargoIntakeSubsystem.ScoringPower.SLOW);
		}
		else if(dc.getPOVLeft()){
			mCargoIntakeSubsystem.scoreCargo(CargoIntakeSubsystem.ScoringPower.FAST);
		}
		else if(dc.getPOVUp()){
			mCargoIntakeSubsystem.scoreCargo(CargoIntakeSubsystem.ScoringPower.IRRESPONSIBLE);
		}
		else if(dc.getButtonLB()){
			mCargoIntakeSubsystem.scoreCargo(CargoIntakeSubsystem.ScoringPower.MEDIUM);
		}

		mCompressorControlSubsystem.setOverride(oc.getButtonY());

		wasB = dc.getButtonB();
		wasRightDPad = dc.getPOVRight();
	}
	public void resetAutoLatch(){
	    autoInTeleOp.resetLatch();
    }
	public void autoRunCommands(){
		autoInTeleOp.update(autoController.getButtonBack());
	}
	public boolean autoInTeleOpOn(){
		return autoInTeleOp.returnStatus();
	}
}
