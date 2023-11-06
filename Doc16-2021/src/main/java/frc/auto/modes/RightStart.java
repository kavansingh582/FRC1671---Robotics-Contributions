package frc.auto.modes;

import java.util.Arrays;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.FollowTrajectoryAction;
import frc.auto.actions.IntakeBallAction;
import frc.auto.actions.ParallelAction;
import frc.auto.actions.RotateTurntable;
import frc.auto.actions.SeriesAction;
import frc.auto.actions.ShootAction;
import frc.robot.Constants;

public class RightStart extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        // RotateTurntable startRotate =  new RotateTurntable(30., 2.);
        // ParallelAction startShot = new ParallelAction(Arrays.asList(new ShootAction(1.0, 5.), new IntakeBallAction(0.6, 5.)));
        // ParallelAction goIntake = new ParallelAction(Arrays.asList(new FollowTrajectoryAction(Constants.rightPath, false), new IntakeBallAction(0.6, 10.)));
        // FollowTrajectoryAction comeBack = new FollowTrajectoryAction(Constants.rightPath, true);
        // ParallelAction endShot = new ParallelAction(Arrays.asList(new ShootAction(1.0, 5.0), new IntakeBallAction(0.6, 5.0)));

        // runAction(new SeriesAction(Arrays.asList(startRotate, startShot, goIntake, comeBack, endShot)));
    }
    
}