package org.usfirst.frc.team1806.robot.auto.sequences;

import org.usfirst.frc.team1806.robot.auto.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.actions.ExtendSquid;
import org.usfirst.frc.team1806.robot.auto.actions.OpenSquid;

public class GrabHatchPanel extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException{
        //Path from vision
        runAction(new ExtendSquid(false));
        runAction(new OpenSquid(false));
        //Possible path to move out from feeder station
    }
}
