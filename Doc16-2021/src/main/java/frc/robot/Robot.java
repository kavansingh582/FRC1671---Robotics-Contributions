/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import trajectory_lib.*;
import frc.auto.AutoModeExecuter;
import frc.auto.actions.IntakeBallAction;
import frc.auto.modes.CenterStart;
import frc.auto.modes.DoNothingMode;
import frc.auto.modes.JustFollowATrajectory;
import frc.auto.modes.JustShoot;
import frc.auto.modes.LeftStart;
import frc.auto.modes.MoveOffLine;
import frc.auto.modes.RightStart;
import frc.auto.modes.TurnInPlace;
import frc.loops.*;
import frc.util.*;
import frc.subsystems.*;
import frc.subsystems.LED;
import frc.subsystems.Flywheel.State;
import frc.subsystems.Intake.IntakeState;
import frc.subsystems.Turntable.TurntableState;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot{

  private AutoModeExecuter autoModeExecuter = new AutoModeExecuter();
  private Drive drive = Drive.getInstance();
  private Flywheel flywheel = Flywheel.getInstance();
  private Intake intake = Intake.getInstance();
  private Turntable turntable = Turntable.getInstance();
  //private AGGG gadget = AGGG.getInstance();
  //private ControlPanel controlPanel = ControlPanel.getInstance();
  private Limelight limelight = Limelight.getInstance();

  private final SubsystemManager subsystemManager = new SubsystemManager(Arrays.asList(Drive.getInstance(), Flywheel.getInstance(), Intake.getInstance(), Turntable.getInstance()));

  Looper internalLooper = new Looper();
  
  private static XboxController base = new XboxController(0);
  private static XboxController co = new XboxController(1);

  private static AxisGreater baseRightTrigger = new AxisGreater(base, 3, 0.3);
  private static AxisGreater baseLeftTrigger = new AxisGreater(base, 2, 0.3);

  private static AxisGreater coRightTrigger = new AxisGreater(co, 3, 0.3);
  private static AxisGreater coLeftTrigger = new AxisGreater(co, 2, 0.3);

  private ArcadeDriveHelper arcadeDriveHelper = new ArcadeDriveHelper();
  private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.1);
  private SlewRateLimiter turnRateLimiter = new SlewRateLimiter(0.2);
  private SlewRateLimiter turntableRateLimiter = new SlewRateLimiter(0.2);

  private SlewRateLimiter gadgetLimiter = new SlewRateLimiter(0.5);
  //PID turntablePID = new PID(0., 0., 0., -1.0, 1.0);

  double teleopStartTime;
  double testStartTime;

  public enum AutoMode{
    DO_NOTHING,
    FOLLOW_PATH,
    CENTER_START,
    LEFT_START,
    RIGHT_START,
    TURN_IN_PLACE,
    JUST_SHOOT,
    MOVE_OFF_LINE
  }


  private AutoMode autoMode = AutoMode.DO_NOTHING;

  public Robot() {
    CrashTracker.logRobotConstruction();
  }

  public void zeroSensors() {
    subsystemManager.zeroSensors();
  } 

  public void allPeriodic() {
    subsystemManager.outputToSmartDashboard();
    subsystemManager.writeToLog();
    internalLooper.outputToSmartDashboard();
  }

//---------S T A R T  O F  O P E R A T O R  C O N T R O L S---------
  //Intake.IntakeState wantedIntakeState = intake.new IntakeState();
  public double flywheelVel = 0.0;
  public void operatorControls() {
    //D R I V E
    DriveSignal signal;
    double throttle, turn;

    throttle = slewRateLimiter.limit(base.getY(Hand.kLeft));
    turn = turnRateLimiter.limit(base.getX(Hand.kRight));
    signal = arcadeDriveHelper.arcadeDrive(throttle, -turn);

    drive.setOpenLoop(signal);

    //F L Y W H E E L
    Flywheel.FlywheelState wantedFlywheelState = flywheel.new FlywheelState();
    if(co.getAButtonPressed()){
      //flywheelVel = 0.72;
      flywheel.setState(Flywheel.State.SPIN_UP);
    }else if(co.getBButtonPressed()){
      flywheelVel = 0.8;
    }else if(co.getYButtonPressed()){
      flywheelVel = 1.0;
    }else if(co.getXButtonPressed()){
      flywheelVel = 0.0;
    }
    wantedFlywheelState.vel = flywheelVel;

    //I N T A K E
    Intake.IntakeState wantedIntakeState = intake.new IntakeState();
    if(co.getBumper(Hand.kRight)){
      wantedIntakeState.beltVel = 0.5;
      wantedIntakeState.intakeBarVel = 0.5;
      wantedIntakeState.hopperVel = 0.5;
    }
    if(coRightTrigger.get()){
      wantedIntakeState.beltVel = 1.0;
    }
    if(co.getBumper(Hand.kLeft)){
      wantedIntakeState.beltVel = -0.5;
      wantedIntakeState.intakeBarVel = -0.5;
      wantedIntakeState.hopperVel = -0.5;
      wantedFlywheelState.vel = -0.2;
    }

    intake.setWantedState(wantedIntakeState);

    //T U R N T A B L E
    if(coLeftTrigger.get()){
      turntable.setState(TurntableState.LIMELIGHT_TARGETING);
      wantedFlywheelState.vel = flywheel.calculateFlywheelVel();
    }else{
      double turntableSignal = 0;
      if (Math.abs(co.getX(Hand.kRight)) > 0.2) {
        turntableSignal = co.getX(Hand.kRight) * .5;
      } else {
        turntable.setBrakeMode(true);
      }
      turntable.setOpenLoop(-turntableSignal);
      //wantedFlywheelState.vel = 0.0;
    }

    flywheel.setWantedState(wantedFlywheelState);
  }
//---------E N D  O F  O P E R A T O R  C O N T R O L S---------

  @Override
  public void robotInit() {
    try {
      subsystemManager.registerEnabledLoops(internalLooper);
    } catch(Throwable t) {
      CrashTracker.logThrowableCrash(t);
			throw t;
    }
    zeroSensors();
  }
  
//DISABLED
  @Override
  public void disabledInit() {
    try {
      CrashTracker.logDisabledInit();
			internalLooper.stop();
      subsystemManager.stop();
      drive.setOpenLoop(DriveSignal.NEUTRAL);
      slewRateLimiter.reset();
      turnRateLimiter.reset();
      drive.setBrakeMode(false);
      zeroSensors();
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
  }

  @Override
  public void disabledPeriodic() {
    allPeriodic();
    drive.setOpenLoop(DriveSignal.NEUTRAL);
    if(base.getBackButtonPressed()){
      autoMode = AutoMode.FOLLOW_PATH;
    }
    if(base.getStartButtonPressed()){
      autoMode = AutoMode.TURN_IN_PLACE;
    }
    if(base.getBumperPressed(Hand.kRight)){
      autoMode = AutoMode.JUST_SHOOT;
    }
    if(base.getBumperPressed(Hand.kLeft)){
      autoMode = AutoMode.MOVE_OFF_LINE;
    }

    if(base.getXButtonPressed()){
      autoMode = AutoMode.LEFT_START;
    }
    if(base.getAButtonPressed()){
      autoMode = AutoMode.CENTER_START;
    }
    if(base.getBButtonPressed()){
      autoMode = AutoMode.RIGHT_START;
    }

    if(co.getAButtonPressed()){
      autoMode = AutoMode.DO_NOTHING;
    }
    System.out.println(autoMode);
    //SmartDashboard.putNumber("distance", limelight.getDistance());
    //System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0));
  }

//AUTO
  @Override
  public void autonomousInit() {
    try{
      zeroSensors();
      internalLooper.start();
      CrashTracker.logAutoInit();
      System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());

      autoModeExecuter = new AutoModeExecuter();
      switch(autoMode){
        case DO_NOTHING:
          autoModeExecuter.setAutoMode(new DoNothingMode());
          break;
        case FOLLOW_PATH:
          autoModeExecuter.setAutoMode(new JustFollowATrajectory());
          break;
        case TURN_IN_PLACE:
          autoModeExecuter.setAutoMode(new TurnInPlace());
          break;
        case JUST_SHOOT:
          autoModeExecuter.setAutoMode(new JustShoot());
          break;
        case MOVE_OFF_LINE:
          autoModeExecuter.setAutoMode(new MoveOffLine());
          break;
        case LEFT_START:
          autoModeExecuter.setAutoMode(new LeftStart());
          break;
        case CENTER_START:
          autoModeExecuter.setAutoMode(new CenterStart());
          break;
        case RIGHT_START:
          autoModeExecuter.setAutoMode(new RightStart());
          break;
        default:
          System.out.println("something went wrong, doing nothing");
          autoModeExecuter.setAutoMode(new DoNothingMode());
      }

      autoModeExecuter.start();
      System.out.println(autoMode);

    } catch(Throwable t){
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }
 
  @Override
  public void autonomousPeriodic() {
    allPeriodic();
    //Led.colorCycle(1.26);
  }

//TELEOP
  @Override
  public void teleopInit() {
    try {
      flywheelVel = 0.;
			CrashTracker.logTeleopInit();
			internalLooper.start();
		  drive.setOpenLoop(DriveSignal.NEUTRAL);
			drive.setBrakeMode(false);
      slewRateLimiter.reset();
      turnRateLimiter.reset();
      zeroSensors();
      teleopStartTime = Timer.getFPGATimestamp();
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
  }

  @Override
  public void teleopPeriodic() {
    try {
      operatorControls();
      SmartDashboard.putNumber("distance", limelight.getDistance());

    } catch(Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
    allPeriodic();
  }


  @Override
  public void testInit() {
    drive.setOpenLoop(DriveSignal.NEUTRAL);
		drive.setBrakeMode(false);
    internalLooper.start();
    slewRateLimiter.reset();
    turnRateLimiter.reset();
    testStartTime = Timer.getFPGATimestamp();
  }

  double timeout = 0.5;
  double aTime;
  @Override
  public void testPeriodic() {
    allPeriodic();
    /*
    Intake.IntakeState wantedIntakeState = intake.new IntakeState();
    Flywheel.FlywheelState wantedFlywheelState = flywheel.new FlywheelState();
    if(co.getAButtonPressed()){
      aTime = Timer.getFPGATimestamp();
    }else if(co.getXButtonPressed()){
      flywheelVel = 0.0;
    }

    if(aTime - Timer.getFPGATimestamp() < 0.5){
      wantedIntakeState.beltVel = -0.6;
    }else{
      flywheelVel = 0.5;
      wantedIntakeState.beltVel = 0.6;
    }

    wantedFlywheelState.vel = flywheelVel;
    flywheel.setWantedState(wantedFlywheelState);
    intake.setWantedState(wantedIntakeState);
    */



    /*
    double turntableSignal = 0;
    if (Math.abs(co.getX(Hand.kRight)) > 0.8) {
      turntableSignal = 15.0 * turnRateLimiter.limit(co.getX(Hand.kRight)); //works but needs a lot of work
    } else {
      turntable.setBrakeMode(true);
    }

    if(coLeftTrigger.get()){
      
    }
    turntable.setOpenLoop(-turntableSignal);
    */
  }

}
