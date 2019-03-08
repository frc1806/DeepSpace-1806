package org.usfirst.frc.team1806.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.SquidSubsystem;

public class CloseSquid implements Action {

    private boolean instant;

    public CloseSquid(boolean _instant){
        instant = _instant;
    }

    SquidSubsystem mSquidSubsystem = SquidSubsystem.getInstance();

    @Override
    public boolean isFinished() {
        return !mSquidSubsystem.isOpen() && Timer.getFPGATimestamp() >= Constants.kCloseSquidTimeToWait || instant;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        Timer.getFPGATimestamp();
        mSquidSubsystem.closeSquid();
    }
}