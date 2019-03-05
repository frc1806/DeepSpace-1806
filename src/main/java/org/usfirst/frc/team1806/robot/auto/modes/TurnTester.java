package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.*;
import org.usfirst.frc.team1806.robot.auto.paths.DumbMode;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class TurnTester extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {

        runAction(new ResetPoseFromPathAction(new DumbMode()));
        runAction(new WaitAction(1));
        runAction(new TurnTowardsPoint(new Translation2d(0,1)));
        runAction(new WaitAction(1));
        runAction(new TurnTowardsPoint(new Translation2d(0,-1)));

    }
}
