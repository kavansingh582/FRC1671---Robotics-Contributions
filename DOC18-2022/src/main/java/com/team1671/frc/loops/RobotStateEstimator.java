package com.team1671.frc.loops;

import com.team1671.frc.RobotState;
import com.team1671.frc.subsystems.Swerve;

public class RobotStateEstimator implements Loop{
	private static RobotStateEstimator instance = null;
	public static RobotStateEstimator getInstance(){
		if(instance == null)
			instance = new RobotStateEstimator();
		return instance;
	}
	
	RobotStateEstimator(){
	}
	
	RobotState robotState = RobotState.getInstance();
	Swerve swerve;

	@Override
	public void onStart(double timestamp) {
		swerve = Swerve.getInstance();
	}

	@Override
	public void onLoop(double timestamp) {
		//robotState.addObservations(timestamp, swerve.getPose(), swerve.getVelocity(), turret.getRotation());
	}

	@Override
	public void onStop(double timestamp) {
		
	}

}
