/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1806.robot;

import org.usfirst.frc.team1806.robot.subsystems.*;
import org.usfirst.frc.team1806.robot.util.CheesyDriveHelper;
import org.usfirst.frc.team1806.robot.util.Latch;
import org.usfirst.frc.team1806.robot.util.XboxController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class

OI {
	private DriveTrainSubsystem mDriveTrainSubsystem = DriveTrainSubsystem.getInstance();
	private SquidSubsystem mSquidSubsystem = SquidSubsystem.getInstance();
	private CompressorControlSubsystem mCompressorControlSubsystem = CompressorControlSubsystem.getInstance();
	private LiftSubsystem mLiftSubsystem = LiftSubsystem.getInstance();
	private HABinAGoodTime mHabClimber = HABinAGoodTime.getInstance();
	private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
	private XboxController dc = new XboxController(0);
	private XboxController oc = new XboxController(1);
	private XboxController autoController = new XboxController(2);
	private Latch autoInTeleOp = new Latch();
	private Boolean wasSquidExtendButton = false;
	private Boolean wasSquidOpenButton = false;
	private Boolean wasChangeModeButton = false;
	private CargoIntakeSubsystem mCargoIntakeSubsystem = CargoIntakeSubsystem.getInstance();

	public void runCommands(){

		//Controls in both modes
		if(Constants.enableAutoInTeleOp){
			autoInTeleOp.update(autoController.getButtonStart());
		}
		synchronized (mDriveTrainSubsystem) {
			if(!((mDriveTrainSubsystem.getmDriveStates() == DriveTrainSubsystem.DriveStates.DRIVE_TO_STALL || Robot.mSequenceState == Robot.SequenceState.VISION) && Robot.mSequenceState.isActive()))
			{
				mDriveTrainSubsystem.setOpenLoop(mCheesyDriveHelper.cheesyDrive(
						dc.getLeftJoyY(), dc.getRightJoyX(), dc.getButtonRB() , mDriveTrainSubsystem.isHighGear()));
			}
			mDriveTrainSubsystem.driveToStall(oc.getButtonA());
		}

		if(FeatureFlags.FF_LIFT_TILT){
			synchronized (mLiftSubsystem){
				if (oc.getButtonRB()){
					mLiftSubsystem.standLiftUp();
				}
				if (oc.getButtonLB()){
					mLiftSubsystem.leanLiftBack();
				}
			}
		}



		/*
		if(oc.getPOVUp()){
			//mHabClimber.goToSetpoint(HABinAGoodTime.ClimbPosition.EXTENSION_LIMIT);
		}
		if(dc.getButtonLS()){
			//mHabClimber.goToSetpoint((HABinAGoodTime.ClimbPosition.RETRACTION_LIMIT));
		}
		*/
		//Controls that change based on mode
		switch(Robot.getGamePieceMode()){
			case HATCH_PANEL:
			default:
				dc.rumble(0, 0);
				synchronized (mLiftSubsystem) {
					if(dc.getButtonA()) {
						mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.BOTTOM_LIMIT);
					}
					if(dc.getButtonX()) {
						mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.ROCKET_HATCH_MID);
					}
					if(dc.getButtonY()) {
						mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.ROCKET_HATCH_HIGH);
					}
					if(dc.getButtonB()) {
						mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.BOTTOM_LIMIT);
					}
				}

				synchronized (mSquidSubsystem) {
					if (!wasSquidOpenButton && dc.getRightTrigger() > Constants.kTriggerThreshold) {
						if (mSquidSubsystem.isOpen()) {
							mSquidSubsystem.closeSquid();
						} else {
							mSquidSubsystem.openSquid();
						}
					}



					if (!wasSquidExtendButton && dc.getLeftTrigger() > Constants.kTriggerThreshold) {
						if (mSquidSubsystem.isExtended()) {
							mSquidSubsystem.retractSquid();
						} else {
							mSquidSubsystem.extendSquid();
						}
					}
				}


				//TODO:Beaver controls


				if(dc.getButtonRB() && !wasChangeModeButton){
					Robot.setGamePieceMode(Robot.GamePieceMode.CARGO);
				}

				break;

			case CARGO:
				dc.rumble(0.7, 0.7);
				synchronized (mLiftSubsystem) {
					if(dc.getButtonX()) {
						mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.ROCKET_CARGO_LOW);
					}
					if(dc.getButtonB()) {
						mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.ROCKET_CARGO_MID);
					}
					if(dc.getButtonY()) {
						mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.ROCKET_CARGO_HIGH);
					}
					if(dc.getButtonA()) {
						mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.BOTTOM_LIMIT);
					}
					if(dc.getRightTrigger() >= Constants.kTriggerThreshold || dc.getButtonStart()) {
						mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.SHIP_CARGO);
					}
				}

				synchronized (mCargoIntakeSubsystem) {


					if (dc.getLeftTrigger() > Constants.kTriggerThreshold) {
						mCargoIntakeSubsystem.intakeCargo();
					} else if (dc.getPOVDown()) {
						mCargoIntakeSubsystem.scoreCargo(CargoIntakeSubsystem.ScoringPower.MEDIUM);
					} else if (dc.getPOVLeft()) {
						mCargoIntakeSubsystem.scoreCargo(CargoIntakeSubsystem.ScoringPower.FAST);
					} else if (dc.getPOVUp()) {
						mCargoIntakeSubsystem.scoreCargo(CargoIntakeSubsystem.ScoringPower.IRRESPONSIBLE);
					} else if (dc.getButtonLB()) {
						mCargoIntakeSubsystem.scoreCargo(CargoIntakeSubsystem.ScoringPower.SLOW);
					} else {
						mCargoIntakeSubsystem.stop();
					}

				}

				if(dc.getButtonRB() && !wasChangeModeButton){
					Robot.setGamePieceMode(Robot.GamePieceMode.HATCH_PANEL);
				}

				break;
		}
		if(dc.getButtonBack() || oc.getPOVDown()){
			Robot.RetractAll();
		}

		if(Math.abs(oc.getLeftJoyY()) > 0.2){
			mLiftSubsystem.manualMode(-oc.getLeftJoyY());
		}
		else if (mLiftSubsystem.returnLiftStates() == LiftSubsystem.LiftStates.MANUAL_CONTROL){
			mLiftSubsystem.manualMode( 0);
		}

		if(Math.abs(oc.getRightJoyY()) > 0.2){
			mHabClimber.manualHandler(true, oc.getRightJoyY(), oc.getRightJoyY() );
		}
		else if (mHabClimber.getmClimbStates() == HABinAGoodTime.ClimbStates.MANUAL_CONTROL) {
			mHabClimber.manualHandler(true, 0, 0);
		}


		mCompressorControlSubsystem.setOverride(oc.getButtonY());

		wasSquidExtendButton = dc.getLeftTrigger() > Constants.kTriggerThreshold;
		wasChangeModeButton = dc.getButtonRB();
		wasSquidOpenButton = dc.getRightTrigger() > Constants.kTriggerThreshold;

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
	public boolean getAutomatedSequenceButton() {
		return dc.getButtonLB();
	}
	public boolean getDisableAutoButton() {
		return oc.getButtonB();
	}


	public Robot.SequenceState getAutomatedSequenceMode() {
		return Robot.SequenceState.VISION; //TODO FIXXXX
	}
}
