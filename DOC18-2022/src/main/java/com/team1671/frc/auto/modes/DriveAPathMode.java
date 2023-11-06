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

public class DriveAPathMode extends AutoModeBase {
    Superstructure s;
    //Swerve swerve;

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.testPath);
    }
    
    public DriveAPathMode(){
        s = Superstructure.getInstance();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();
        //runAction(new ResetPoseAction(Constants.testStartPose));
        runAction(new SetTrajectoryAction(trajectories.testPath, 0.0, 1.0));
        runAction(new WaitToFinishPathAction());
        System.out.println("Auto mode finished in " + currentTime() + " seconds");

    }
}
