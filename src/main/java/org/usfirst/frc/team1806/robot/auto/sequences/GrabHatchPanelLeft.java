package org.usfirst.frc.team1806.robot.auto.sequences;

import org.usfirst.frc.team1806.robot.auto.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.actions.ExtendSquid;
import org.usfirst.frc.team1806.robot.auto.actions.OpenSquid;

public class GrabHatchPanelLeft extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        //Path from vision
        runAction(new ExtendSquid(false));
        runAction(new OpenSquid(false));
        //Reset odometry to left feeder station
        //Possible path to move out from feeder station
    }
}
