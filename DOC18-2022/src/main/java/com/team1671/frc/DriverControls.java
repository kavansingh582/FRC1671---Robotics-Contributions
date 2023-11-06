/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1671.frc;

import java.util.Arrays;

import com.team1671.frc.loops.Candle;
import com.team1671.frc.loops.LimelightProcessor;
import com.team1671.frc.loops.Loop;
import com.team1671.frc.subsystems.ActuatingHood;
import com.team1671.frc.subsystems.Climber;
import com.team1671.frc.subsystems.Feeder;
import com.team1671.frc.subsystems.Intake;
import com.team1671.frc.subsystems.Shooter;
import com.team1671.frc.subsystems.SubsystemManager;
import com.team1671.frc.subsystems.Superstructure;
import com.team1671.frc.subsystems.Swerve;
import com.team1671.frc.subsystems.Turret;
import com.team1671.frc.subsystems.Feeder.State;
import com.team1671.io.PS5;
import com.team1671.lib.util.Util;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

/**
 * A class to assign controller inputs to robot actions
 */
public class DriverControls implements Loop {

    private static DriverControls instance = null;

    public static DriverControls getInstance() {
        if (instance == null)
            instance = new DriverControls();
        return instance;
    }

    PS5 driver, coDriver;

    private Swerve swerve;
    private Intake intake;
    private Turret turret;
    private Feeder feeder;
    private Shooter shooter;
    private Candle candle;
    private Climber climber;
    private ActuatingHood actuatingHood;


    private Superstructure s;

    private SubsystemManager subsystems;
    public SubsystemManager getSubsystems(){ return subsystems; }

    private RobotState robotState;

    private LimelightProcessor limelight;

	private boolean flickRotation = false;
    private boolean robotCentric = false;
    private boolean baseReadyToClimb = false;
    private boolean coReadyToClimb = false;
    public boolean setClimbInit = false;
    private boolean climbRungs = false;


    double output = 0.5;
    int temp = 0;

    private boolean inAuto = true;
    public void setAutoMode(boolean auto) {
        inAuto = auto;
    }

    public boolean getInAuto() {
        return inAuto;
    }

    public DriverControls() {
        driver = new PS5(0);
        coDriver = new PS5(1);
        driver.setDeadband(0.0);
        coDriver.setDeadband(0.25);
		coDriver.rightBumper.setLongPressDuration(1.0);

        swerve = Swerve.getInstance();
        turret = Turret.getInstance();
        intake = Intake.getInstance();
        feeder = Feeder.getInstance();
        shooter = Shooter.getInstance();
        candle = Candle.getInstance();
        climber = Climber.getInstance();
        actuatingHood = ActuatingHood.getInstance();


        s = Superstructure.getInstance();

        subsystems = new SubsystemManager(
				Arrays.asList(turret, climber, feeder, intake, shooter, actuatingHood, swerve, s));

        robotState = RobotState.getInstance();

        limelight = LimelightProcessor.getInstance();
    }

    @Override
    public void onStart(double timestamp) {
        if(inAuto) {
            swerve.zeroSensors();
            swerve.setNominalDriveOutput(0.0);
            swerve.requireModuleConfiguration();
            s.enableCompressor(false);
        } else {
            s.enableCompressor(true);
            swerve.setNominalDriveOutput(0.0);
            swerve.set10VoltRotationMode(false);
        }
    }

    @Override
    public void onLoop(double timestamp) {
        if(inAuto) {
            /*if(s.swerve.getState() == Swerve.ControlState.VISION_PID){
            }else{
            }*/
        } else {
            driver.update();
            coDriver.update();
            twoControllerMode();
        }
    }

    @Override
    public void onStop(double timestamp) {
        subsystems.stop();
    }

    private void twoControllerMode() {
        double swerveYInput = (driver.getLeftX());
        double swerveXInput = (-driver.getLeftY());
        double swerveRotationInput = (flickRotation ? 0.0 : driver.getRightX());
        double turretInput = coDriver.getRightX();
        double climbInput = -coDriver.getLeftY();



       

        swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, robotCentric, driver.leftTrigger.isBeingPressed());

        if (flickRotation) {
            swerve.updateControllerDirection(new Translation2d(-driver.getRightY(), driver.getRightX()));
            if (!Util.epsilonEquals(
            Util.placeInAppropriate0To360Scope(swerve.getTargetHeading(),
            swerve.averagedDirection.getDegrees()),
            swerve.getTargetHeading(), swerve.rotationDivision / 2.0)) {
                swerve.rotate(swerve.averagedDirection.getDegrees());
            }
        }


        //Base controls
        if (driver.circleButton.isBeingPressed()){
            swerve.rotate(90);
        } if(driver.xButton.isBeingPressed()){
            swerve.rotate(180);
        } if(driver.triangleButton.isBeingPressed()){
            swerve.rotate(0);
        } if(driver.squareButton.isBeingPressed()){
            swerve.rotate(270);
        } if(driver.menuButton.isBeingPressed()) {
            //robot centric
            robotCentric = true;
        }else{
            robotCentric = false;
        } if(driver.shareButton.shortReleased() || driver.shareButton.longPressed()) {
            swerve.temporarilyDisableHeadingController();
            swerve.zeroSensors(new Pose2d());
            swerve.resetAveragedDirection();
        } if (driver.rightTrigger.isBeingPressed()){
            swerve.lockDrivePositionRequest();
        }
        if (driver.leftTrigger.wasActivated()){
            //ready to climb LEDS
            //s.createCandleStripe(71, 0,0.0, 255,255,0, 0, 71,"One ready to climb");
            baseReadyToClimb = true;
            //swerve.setMaxSpeed(1.0);
        }else{
            //swerve.setMaxSpeed(.6);
        }

        if (driver.rightBumper.isBeingPressed()){
            //smaller goal orientation
            swerve.rotate(21);
        } if (driver.leftBumper.isBeingPressed()){
            //larger goal orientation
            swerve.rotate(159);
        }



        //CO CONTROLS

        if (baseReadyToClimb && coReadyToClimb){
            System.out.println("climbing");
            if(!setClimbInit){
                //System.out.println("climb mode");
                s.bruhWeClimbin();
                s.candleSetClimbing(true);
                setClimbInit=true;
                climbRungs = true;
            }
            
             if (Math.abs(climbInput) > 0){
                climber.setOpenLoop(climbInput);
                }else if(climbInput == 0 && climber.getState() == Climber.State.OPEN_LOOP){
                climber.setOpenLoop(0.0);
                }


            if (coDriver.rightTrigger.wasActivated()){
                s.ClimberExtend(20);
            }else if (coDriver.leftTrigger.wasActivated()){
                s.ClimberRetract();
            }else if (coDriver.POV0.wasActivated()){
               s.ClimberExtendPiston(true);
                climbRungs = false;
            }else if (coDriver.POV180.wasActivated()){
                s.ClimberExtendPiston(false);
                climbRungs = true;
            }
            
            if(coDriver.shareButton.shortReleased()){
                climber.zeroSensors();
            }

            if(coDriver.menuButton.wasActivated()){
                s.ClimberExtendPiston(false);
                climbRungs = true;
                s.candleSetClimbing(false);
                baseReadyToClimb = false;
                coReadyToClimb = false;
                setClimbInit = false;
            }

            

        }else{

            //teleop controls

             //all the manual stuff
            if (Math.abs(turretInput) > 0){
            turret.setOpenLoop(turretInput);
            }else if(turretInput == 0 && turret.getState() == Turret.State.OPEN_LOOP){
            turret.setOpenLoop(0.0);
            }

            if (coDriver.triangleButton.wasReleased()){
                s.intakeStopState();
            }else if (coDriver.triangleButton.wasActivated()){
                s.intakeReverse();
            }else if (coDriver.xButton.wasActivated()){
                s.intakeState();
                s.candleSetIsIntaking(true);
            }else if (coDriver.xButton.wasReleased()){
                s.intakeStopState();
                s.candleSetIsIntaking(false);
            }
            if (coDriver.circleButton.isBeingPressed()){
                s.neutralState();
                System.out.println("neutral state");
            }
            if (coDriver.POV0.wasActivated()){
                s.servoFarRequest();
            }else if (coDriver.POV180.wasActivated()){
                s.servoCloseRequest();
            }
            if (coDriver.POV90.wasActivated()){
                //s.turretTurn(-159);
            }else if (coDriver.POV270.wasActivated()){
                //s.turretTurn(21);
            }
            if (coDriver.squareButton.wasActivated()){
                coReadyToClimb = true;
            }



            if (coDriver.rightTrigger.wasActivated()){
                s.shootingState();
                //s.testShootingState(1000);
                //s.ShootBalls();
            }
            if(coDriver.leftTrigger.wasActivated()){
                s.visionLockOnRequest();
                //s.goodVisionLockOnRequest();
            }
            if(coDriver.leftTrigger.wasReleased()){
                //s.feedstopstate();
            }
            if(coDriver.leftBumper.wasActivated()){
                s.shootingBottomState();
            }

            if(coDriver.rightBumper.wasActivated()){
                s.shootingTopState();
            }

            //if(coDriver.POV90.wasActivated()){
                //s.feedstate();

            //}if (coDriver.POV90.wasReleased()){
                //s.feedstopstate();
            //}

        }

    }
}
