/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1671.frc;

import com.team1671.frc.auto.AutoModeBase;
import com.team1671.frc.auto.AutoModeExecuter;
import com.team1671.frc.auto.SmartDashboardInteractions;
import com.team1671.frc.auto.modes.DriveAPathMode;
import com.team1671.frc.loops.Candle;
import com.team1671.frc.loops.LimelightProcessor;
import com.team1671.frc.loops.Looper;
import com.team1671.frc.loops.QuinticPathTransmitter;
import com.team1671.frc.loops.RobotStateEstimator;
import com.team1671.frc.loops.Candle.State;
import com.team1671.frc.subsystems.Pigeon;
import com.team1671.frc.subsystems.SubsystemManager;
import com.team1671.frc.subsystems.Superstructure;
import com.team1671.lib.util.CrashTracker;
import com.team1671.lib.util.Logger;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.wpilib.TimedRobot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

	private Pigeon pigeon;
	private Superstructure s;
	private SubsystemManager subsystems;

	private AutoModeExecuter autoModeExecuter = null;
	private TrajectoryGenerator generator = TrajectoryGenerator.getInstance();
	private QuinticPathTransmitter qTransmitter = QuinticPathTransmitter.getInstance();
	private SmartDashboardInteractions smartDashboardInteractions = new SmartDashboardInteractions();


	private Looper enabledLooper = new Looper();
	private Looper disabledLooper = new Looper();

	private RobotState robotState = RobotState.getInstance();

	private DriverStation ds = DriverStation.getInstance();

	private DriverControls driverControls;	
	
	private Candle candle;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		driverControls = DriverControls.getInstance();

		//pigeon = Pigeon.getInstance();
		s = Superstructure.getInstance();
		subsystems = driverControls.getSubsystems();
		candle = Candle.getInstance();

		Logger.clearLog();

		enabledLooper.register(driverControls);
		enabledLooper.register(RobotStateEstimator.getInstance());
		enabledLooper.register(QuinticPathTransmitter.getInstance());
		enabledLooper.register(LimelightProcessor.getInstance());
		enabledLooper.register(Candle.getInstance());
		disabledLooper.register(RobotStateEstimator.getInstance());
		disabledLooper.register(QuinticPathTransmitter.getInstance());
		disabledLooper.register(LimelightProcessor.getInstance());
		disabledLooper.register(Candle.getInstance());
		subsystems.registerEnabledLoops(enabledLooper);
		subsystems.registerDisabledLoops(disabledLooper);
		
		//s.swerve.zeroSensors(new Pose2d());
		//s.swerve.stop()

		generator.generateTrajectories();

		//paths stored in trajectory generator
		AutoModeBase auto = new DriveAPathMode();
		smartDashboardInteractions.initWithDefaults();
		
		qTransmitter.addPaths(auto.getPaths());
		System.out.println("Total path time: " + qTransmitter.getTotalPathTime(auto.getPaths()));
	}

	@Override
	public void robotPeriodic() {
		subsystems.outputToSmartDashboard();
		robotState.outputToSmartDashboard();
		//pigeon.outputToSmartDashboard();
		SmartDashboard.putBoolean("Enabled", ds.isEnabled());
		SmartDashboard.putNumber("Match time", ds.getMatchTime());
		Settings.update();
	}

	@Override
	public void autonomousInit() {
		try {
			if (autoModeExecuter != null) autoModeExecuter.stop();

			driverControls.setAutoMode(true);
			//s.candleDefaultLEDRequest();
			disabledLooper.stop();
			enabledLooper.start();
			

			SmartDashboard.putBoolean("Auto", true);

			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode(smartDashboardInteractions.getSelectedAutoMode());
			autoModeExecuter.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopInit() {
		try {
			driverControls.setAutoMode(false);
			//s.candleDefaultLEDRequest();
			disabledLooper.stop();
			enabledLooper.start();
			//ledLooper.stop();
			SmartDashboard.putBoolean("Auto", false);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}


	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		try {
			enabledLooper.outputToSmartDashboard();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();
			enabledLooper.stop();
			subsystems.stop();
			disabledLooper.start();
			s.candleSetIsIntaking(false);
			s.candleSetClimbing(false);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		try {
			disabledLooper.outputToSmartDashboard();
			Settings.update();
			//s.swerve.zeroModuleAngles();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {

	}

}
