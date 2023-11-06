package com.team1671.frc.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team1671.frc.Constants;
import com.team1671.frc.auto.AutoModeBase;
import com.team1671.frc.auto.AutoModeEndedException;
import com.team1671.frc.auto.actions.ResetPoseAction;
import com.team1671.frc.auto.actions.SetTrajectoryAction;
import com.team1671.frc.auto.actions.WaitToFinishPathAction;
import com.team1671.frc.subsystems.Superstructure;
import com.team1671.frc.subsystems.Swerve;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

public class DriveAPathBackwardsMode extends AutoModeBase {
    Superstructure s;
    Swerve swerve;

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.backTestPath);
    }
    
    public DriveAPathBackwardsMode(){
        s = Superstructure.getInstance();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();
        System.out.println("doing drive a path");
        runAction(new ResetPoseAction(Constants.testStartPose));
        System.out.println("reset pose");
        runAction(new SetTrajectoryAction(trajectories.backTestPath, 0, 0.1));
        System.out.println("driving path");
        runAction(new WaitToFinishPathAction());
        System.out.println("Auto mode finished in " + currentTime() + " seconds");

    }
}
