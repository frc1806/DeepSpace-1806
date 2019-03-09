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
	private Boolean wasSquidExtendButton = false;
	private Boolean wasGrabHatchLeftButton = false;
	private Boolean wasGrabHatchRightButton = false;
	private Boolean wasScoreHatchButton = false;
	private Boolean wasScoreCargoButton = false;
	private Boolean wasChangeModeButton = false;
	private Boolean wasChangeControlModeButton = false;
	private CargoIntakeSubsystem mCargoIntakeSubsystem = CargoIntakeSubsystem.getInstance();

	public void runCommands(){


			//Controls in both modes
			if (Constants.enableAutoInTeleOp) {
				autoInTeleOp.update(autoController.getButtonStart());
			}
			synchronized (mDriveTrainSubsystem) {
				if (dc.getRightTrigger() > .2) {
					mDriveTrainSubsystem.setCreepMode(mCheesyDriveHelper.cheesyDrive(
							dc.getLeftJoyY(), dc.getRightJoyX(), dc.getButtonRB(), mDriveTrainSubsystem.isHighGear()));
				} else {
					mDriveTrainSubsystem.setOpenLoop(mCheesyDriveHelper.cheesyDrive(
							dc.getLeftJoyY(), dc.getRightJoyX(), dc.getButtonRB(), mDriveTrainSubsystem.isHighGear()));
				}
			}

			if (dc.getButtonBack() || oc.getPOVDown()) {
				Robot.RetractAll();
			}

			//Controls that change based on mode
			switch (Robot.getGamePieceMode()) {
				case HATCH_PANEL:
				default:
					synchronized (mLiftSubsystem) {
						if (dc.getButtonA()) {
							mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.ROCKET_HATCH_LOW);
						}
						if (dc.getButtonX()) {
							mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.ROCKET_HATCH_MID);
						}
						if (dc.getButtonY()) {
							mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.ROCKET_HATCH_HIGH);
						}
						if (dc.getButtonB()) {
							mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.BOTTOM_LIMIT);
						}
					}

					synchronized (mSquidSubsystem) {
						if (dc.getLeftTrigger() > Constants.kTriggerThreshold) {
							mSquidSubsystem.openSquid();
						}

						if (dc.getButtonLB()) {
							mSquidSubsystem.closeSquid();
						}


						if (!wasSquidExtendButton && dc.getRightTrigger() > Constants.kTriggerThreshold) {
							if (mSquidSubsystem.isExtended()) {
								mSquidSubsystem.retractSquid();
							} else {
								mSquidSubsystem.isExtended();
							}
						}
					}


					//TODO:Beaver controls


					if (dc.getButtonRB() && !wasChangeModeButton) {
						Robot.setGamePieceMode(Robot.GamePieceMode.CARGO);
					}

					break;

				case CARGO:

					synchronized (mLiftSubsystem) {
						if (dc.getButtonA()) {
							mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.ROCKET_CARGO_LOW);
						}
						if (dc.getButtonX()) {
							mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.ROCKET_CARGO_MID);
						}
						if (dc.getButtonY()) {
							mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.ROCKET_CARGO_HIGH);
						}
						if (dc.getButtonB()) {
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

					if (dc.getButtonRB() && !wasChangeModeButton) {
						Robot.setGamePieceMode(Robot.GamePieceMode.HATCH_PANEL);
					}

					break;
			}


		switch (Robot.getControlMode()){
			case OPERATOR_CONTROL:
			default:


				if(!wasChangeControlModeButton && oc.getPOVUp()){
					Robot.setControlMode(Robot.ControlMode.HATCH_VISION_CONTROL);
				}
				break;

			case HATCH_VISION_CONTROL:

				if(!wasGrabHatchLeftButton && oc.getRightTrigger() > Constants.kTriggerThreshold){

				}
				else if(wasGrabHatchLeftButton && oc.getRightTrigger() < Constants.kTriggerThreshold){

				}

				if(!wasGrabHatchRightButton && oc.getLeftTrigger() > Constants.kTriggerThreshold){

				}
				else if(wasGrabHatchRightButton && oc.getLeftTrigger() < Constants.kTriggerThreshold){

				}

				if(!wasScoreHatchButton && oc.getButtonY()){

				}
				else if(wasScoreHatchButton && !oc.getButtonY()){

				}


				if(!wasChangeControlModeButton && oc.getPOVUp()){
					Robot.setControlMode(Robot.ControlMode.CARGO_VISION_CONTROL);
				}
				break;
			case CARGO_VISION_CONTROL:

				if(!wasScoreCargoButton && oc.getButtonX()){

				}
				else if(wasScoreCargoButton && !oc.getButtonX()){

				}

				if(!wasChangeControlModeButton && oc.getPOVUp()){
					Robot.setControlMode(Robot.ControlMode.OPERATOR_CONTROL);
				}
		}



		mCompressorControlSubsystem.setOverride(oc.getButtonY());

		wasSquidExtendButton = dc.getRightTrigger() > Constants.kTriggerThreshold;
		wasGrabHatchLeftButton = oc.getRightTrigger() > Constants.kTriggerThreshold;
		wasGrabHatchRightButton = oc.getLeftTrigger() > Constants.kTriggerThreshold;
		wasScoreHatchButton = oc.getButtonY();
		wasScoreCargoButton = oc.getButtonX();
		wasChangeModeButton = dc.getButtonRB();
		wasChangeControlModeButton = oc.getPOVUp();

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
