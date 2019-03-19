package org.usfirst.frc.team1806.robot.auto.modes;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.*;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.DumbMode;
import org.usfirst.frc.team1806.robot.auto.paths.VisionPath;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class VisionMode extends AutoModeBase {

    boolean done = false;
    @Override
    protected void routine() throws AutoModeEndedException {
        done = false;
        int testX = 72;
        int testY = 12;
        double testAngle = 0;
        VisionPath visPath = new VisionPath(new RigidTransform2d(new Translation2d(testX, testY), Rotation2d.fromRadians(testAngle)));
        runAction(new SwitchToLowPID());
        runAction(new DrivePathAction(visPath));
        runAction(new TurnTowardsPoint(visPath.bayyPose.getTranslation()));
        done = true;
        runAction(new WaitAction(15));

    }

    public boolean getIsDone(){
        return done;
    }
}
