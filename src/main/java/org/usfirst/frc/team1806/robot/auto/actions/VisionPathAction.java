package org.usfirst.frc.team1806.robot.auto.actions;

import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.Action;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.DrivePathAction;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class VisionPathAction implements Action {
    Translation2d odometry;
    VisionPathAction(Translation2d odo) {
        odometry = odo;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        //new DrivePathAction()
    }

    public PathContainer generateVisionPath() {
        return new PathContainer() {
            @Override
            public Path buildPath() {
                return null;
            }

            @Override
            public RigidTransform2d getStartPose() {
                return null;
            }

            @Override
            public boolean isReversed() {
                return false;
            }
        };
    }
}
