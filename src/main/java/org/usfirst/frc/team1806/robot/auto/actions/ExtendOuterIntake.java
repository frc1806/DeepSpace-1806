package org.usfirst.frc.team1806.robot.auto.actions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.CargoIntakeSubsystem;

public class ExtendOuterIntake implements Action {

    private boolean instant;

    public ExtendOuterIntake(boolean _instant){
        instant = _instant;
    }

    CargoIntakeSubsystem mCargoIntakeSubsystem = CargoIntakeSubsystem.getInstance();

    @Override
    public boolean isFinished() {
        return mCargoIntakeSubsystem.isOuterIntakeExtended() || instant;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mCargoIntakeSubsystem.extendOuterIntake();
    }
}
