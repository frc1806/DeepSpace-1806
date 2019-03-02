package org.usfirst.frc.team1806.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.subsystems.SquidSubsystem;

public class ScoreHatchPanel implements Action {

    private boolean instant;
    private double timeToWait;

    public ScoreHatchPanel(boolean _instant){
        instant = _instant;
        timeToWait = 0.5;
    }

    SquidSubsystem mSquidSubsystem = SquidSubsystem.getInstance();

    @Override
    public boolean isFinished() {
        return !mSquidSubsystem.isOpen() && Timer.getFPGATimestamp() >= timeToWait || instant;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
//        mSquidSubsystem.retractSquid();
    }

    @Override
    public void start() {
        Timer.getFPGATimestamp();
        mSquidSubsystem.closeSquid();
    }
}
