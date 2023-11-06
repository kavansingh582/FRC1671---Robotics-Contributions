package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.FollowTrajectoryAction;
import frc.auto.actions.ShootAction;
import frc.robot.Constants;

public class JustShoot extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ShootAction(0.65, 2.5, 0.5)); //shot speed, total time, revup time
        runAction(new FollowTrajectoryAction(Constants.moveOffLinePath, false));
    }
    
}