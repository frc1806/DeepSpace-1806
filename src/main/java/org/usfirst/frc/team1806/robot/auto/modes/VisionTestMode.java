package org.usfirst.frc.team1806.robot.auto.modes;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.*;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.auto.paths.DumbMode;
import org.usfirst.frc.team1806.robot.auto.paths.VisionPath;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class VisionTestMode  extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        int testX = 72;
        int testY = 12;
        double testAngle = 0;
        runAction(new WaitAction(1));
        runAction(new DrivePathAction(new VisionPath(new RigidTransform2d(new Translation2d(testX, testY), Rotation2d.fromRadians(testAngle)))));
        runAction(new TurnToHeading(0));
        runAction(new WaitAction(15));

    }
}
