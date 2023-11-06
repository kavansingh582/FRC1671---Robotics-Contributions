package frc.auto.modes;


import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.FollowTrajectoryAction;
import frc.robot.Constants;

public class CenterStart extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new FollowTrajectoryAction(Constants.rightPath, false));
    }
    
}