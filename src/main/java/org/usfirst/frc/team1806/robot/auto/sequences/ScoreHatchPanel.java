package org.usfirst.frc.team1806.robot.auto.sequences;

import org.usfirst.frc.team1806.robot.auto.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.actions.CloseSquid;
import org.usfirst.frc.team1806.robot.auto.actions.RetractSquid;

public class ScoreHatchPanel extends AutoModeBase {

    @Override
    protected void routine()throws AutoModeEndedException {

        runAction(new CloseSquid(false));
        runAction(new RetractSquid(false));
    }
}
