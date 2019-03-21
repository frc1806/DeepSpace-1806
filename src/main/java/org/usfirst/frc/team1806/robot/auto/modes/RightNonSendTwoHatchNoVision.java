package org.usfirst.frc.team1806.robot.auto.modes;

import org.usfirst.frc.team1806.robot.auto.actions.LiftToHeight;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.CloseSquid;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.ExtendSquid;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.OpenSquid;
import org.usfirst.frc.team1806.robot.auto.actions.SquidActions.RetractSquid;
import org.usfirst.frc.team1806.robot.auto.actions.VisionPathExecuter;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.*;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.DriveStraightPath;
import org.usfirst.frc.team1806.robot.auto.paths.RightSideCloseHatchRocketToFeeder;
import org.usfirst.frc.team1806.robot.auto.paths.RightSideHAB1ToCloseHatchRocket;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.lang.reflect.Array;
import java.util.Arrays;

public class RightNonSendTwoHatchNoVision extends AutoModeBase {
    @Override
    public void routine() throws AutoModeEndedException {
        runAction(new SwitchToHighPID());

        PathContainer startPath = new RightSideHAB1ToCloseHatchRocket();

        //Drive out to close side of rocket
        runAction(new ResetPoseFromPathAction(startPath));
        runAction(new DrivePathAction(startPath));

        //runAction(new TurnTowardsPoint(new Translation2d(210, 10)));

        runAction(new ParallelAction(Arrays.asList(
                new ExtendSquid(),
                new VisionPathExecuter()
        )));
        runAction(new ExtendSquid());
        //At the same time, raise the lift to the high hatch slot while visioning into the bay
        runAction(new VisionPathExecuter());



        //Scoring sequence

        runAction(new SwitchToLowPID());
        runAction(new WaitAction(.2));
        runAction(new DrivePathAction(new DriveStraightPath(4, 25)));
        runAction(new WaitAction(1.5));
        runAction(new CloseSquid());
        runAction(new WaitAction(1));
        runAction(new ParallelAction(Arrays.asList(new RetractSquid(), new DrivePathAction(new DriveStraightPath(-4, 20)))));

        runAction(new SwitchToHighPID());

        //Give some time to lower lift to a safe driving height
        //runAction(new LiftToHeight(LiftSubsystem.LiftPosition.ROCKET_CARGO_LOW, false));

        //Concurrently finish lowering lift shoot backwards towards feeder station, turn towards it
        runAction(new DrivePathAction(new RightSideCloseHatchRocketToFeeder()));
        runAction(new TurnTowardsPoint(new Translation2d(0,41)));

        //vision path
        runAction(new VisionPathExecuter());

        //Collect hatch
        runAction(new ExtendSquid());
        runAction(new WaitAction(.2));
        runAction(new DrivePathAction(new DriveStraightPath(4, 30)));
        runAction(new WaitAction(1));
        runAction(new OpenSquid());
        runAction(new WaitAction(1.5));
        runAction(new ParallelAction(Arrays.asList(new RetractSquid(), new DrivePathAction(new DriveStraightPath(-4, 20)))));


        runAction(new WaitAction(15));
    }

}
