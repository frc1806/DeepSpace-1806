package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.auto.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.*;
import org.usfirst.frc.team1806.robot.auto.paths.DumbMode;
import org.usfirst.frc.team1806.robot.auto.paths.TrainingPath;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.Arrays;

public class TrainingMode extends AutoModeBase {
    /*
    So guys:
    This is gonna maybe be an auto if we don't frick it up. So let's not do that!
    In this auto, we're pre-loaded with a hatch and we're gonna patch up the first level of the right rocket.
    Let's win!
    ~CCP
    */

    @Override
    public void routine() throws AutoModeEndedException {
        PathContainer trainingDealio = new TrainingPath(); //I de-capitalized the variable name. Coding standards. -Dillon
        runAction(new ResetPoseFromPathAction(trainingDealio));
        runAction(new DrivePathAction(trainingDealio));
        runAction(new WaitAction(15));
    }
}
