package org.usfirst.frc.team1806.robot.auto.actions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;

public class Intake implements Action {

    private boolean instant;

    public Intake(boolean _instant){
        instant = _instant;
    }



    @Override
    public boolean isFinished() {
        return instant; //We need a sensor
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {

    }
}
