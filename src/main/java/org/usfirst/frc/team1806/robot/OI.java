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

				mDriveTrainSubsystem.setOpenLoop(mCheesyDriveHelper.cheesyDrive(
						dc.getLeftJoyY(), dc.getRightJoyX(), dc.getButtonRB() , mDriveTrainSubsystem.isHighGear()));
		}

		//Controls that change based on mode
		switch(Robot.getGamePieceMode()){
			case HATCH_PANEL:
			default:
				synchronized (mLiftSubsystem) {

					if(dc.getButtonA()) {
						mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.BOTTOM_LIMIT\
						);
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
					if (!wasSquidOpenButton && dc.getLeftTrigger() > Constants.kTriggerThreshold) {
						if (mSquidSubsystem.isOpen()) {
							mSquidSubsystem.closeSquid();
						} else {
							mSquidSubsystem.openSquid();
						}
					}



					if (!wasSquidExtendButton && dc.getRightTrigger() > Constants.kTriggerThreshold) {
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
				}

				synchronized (mCargoIntakeSubsystem) {
					if (!mLiftSubsystem.isNeedingIntakeOut()) {
						if (dc.getRightTrigger() >= Constants.kTriggerThreshold) {
							mCargoIntakeSubsystem.extendOuterIntake();
						} else {
							mCargoIntakeSubsystem.retractOuterIntake();
						}
					}

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


		mCompressorControlSubsystem.setOverride(oc.getButtonY());

		wasSquidExtendButton = dc.getRightTrigger() > Constants.kTriggerThreshold;
		wasChangeModeButton = dc.getButtonRB();
		wasSquidOpenButton = dc.getLeftTrigger() > Constants.kTriggerThreshold;

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
