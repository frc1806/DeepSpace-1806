package org.usfirst.frc.team1806.robot.auto.actions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.LiftSubsystem;

public class liftToHeight implements Action {
    LiftSubsystem mLiftSubsystem = LiftSubsystem.getInstance();
    @Override
    public boolean isFinished() {
        return mLiftSubsystem.isAtPosition();
    }

    @Override
    public void update() {

    }

    @Override
    public void done()        {

    }

    @Override
    public void start() {
        mLiftSubsystem.goToSetpoint(LiftSubsystem.LiftPosition.TELEOP_HOLD);
    }
}
