package com.team1671.frc.auto.actions;

import com.team1671.frc.subsystems.Superstructure;

public class WaitForSuperstructureAction implements Action{
	private Superstructure superstructure;
	
	public WaitForSuperstructureAction(){
		superstructure = Superstructure.getInstance();
	}

	@Override
	public boolean isFinished() {
		return superstructure.requestsCompleted();
	}

	@Override
	public void start() {		
	}

	@Override
	public void update() {		
	}

	@Override
	public void done() {
	}
	
}
