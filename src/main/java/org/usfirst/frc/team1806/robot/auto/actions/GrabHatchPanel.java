package org.usfirst.frc.team1806.robot.auto.actions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.SquidSubsystem;

public class GrabHatchPanel implements Action {

    private boolean instant;

    public GrabHatchPanel(boolean _instant){
        instant = _instant;
    }

    SquidSubsystem mSquidSubsystem = SquidSubsystem.getInstance();

    @Override
    public boolean isFinished() {
        return mSquidSubsystem.isOpen() || instant;
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
