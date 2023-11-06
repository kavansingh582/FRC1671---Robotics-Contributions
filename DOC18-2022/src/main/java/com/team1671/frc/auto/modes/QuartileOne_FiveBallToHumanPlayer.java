package com.team1671.frc.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team1671.frc.Constants;
import com.team1671.frc.auto.AutoModeBase;
import com.team1671.frc.auto.AutoModeEndedException;
import com.team1671.frc.auto.actions.ResetPoseAction;
import com.team1671.frc.auto.actions.SetTrajectoryAction;
import com.team1671.frc.auto.actions.WaitForSuperstructureAction;
import com.team1671.frc.auto.actions.WaitToFinishPathAction;
import com.team1671.frc.subsystems.Superstructure;
import com.team1671.frc.subsystems.Swerve;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

public class QuartileOne_FiveBallToHumanPlayer extends AutoModeBase {
    Superstructure s;
    Swerve swerve;

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.Q1ToRedBall1, trajectories.RedBall1toBall2, trajectories.RedBall2toBall3, trajectories.RedBall3toBall2);
    }
    
    public QuartileOne_FiveBallToHumanPlayer(){
        s = Superstructure.getInstance();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();
        runAction(new ResetPoseAction(Constants.quartileOneStartingPose));
        s.firingVisionState();
        runAction(new WaitForSuperstructureAction());
        s.intakeState();
        runAction(new SetTrajectoryAction(trajectories.Q1ToRedBall1, -45, 7));
        runAction(new WaitToFinishPathAction());
        runAction(new SetTrajectoryAction(trajectories.RedBall1toBall2, 45, 7));
        runAction(new WaitToFinishPathAction());
        s.intakeStopState();
        s.firingVisionState();
        runAction(new WaitForSuperstructureAction());
        s.intakeState();
        runAction(new SetTrajectoryAction(trajectories.RedBall2toBall3, 45, 7));
        runAction(new WaitToFinishPathAction());
        runAction(new SetTrajectoryAction(trajectories.RedBall3toBall2, 45, 7));
        runAction(new WaitToFinishPathAction());
        s.intakeStopState();
        s.firingVisionState();
        runAction(new WaitForSuperstructureAction());
        System.out.println("Auto mode finished in " + currentTime() + " seconds");

    }
}
