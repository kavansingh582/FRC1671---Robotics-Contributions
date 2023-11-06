package com.team1671.frc.auto.actions;

import com.team1671.frc.subsystems.Swerve;

public class SetTargetHeadingAction extends RunOnceAction{
	double targetHeadingWanted;
	Swerve swerve;
	
	public SetTargetHeadingAction(double targetHeading){
		targetHeadingWanted = targetHeading;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public void runOnce() {
		swerve.setAbsolutePathHeading(targetHeadingWanted);
	}

}
