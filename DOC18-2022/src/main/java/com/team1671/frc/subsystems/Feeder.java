/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1671.frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.team1671.frc.Constants;
import com.team1671.frc.Ports;
import com.team1671.frc.Settings;
import com.team1671.frc.subsystems.requests.Request;
import com.team254.drivers.LazyTalonFX;
import com.team1671.frc.loops.ILooper;
import com.team1671.frc.loops.Loop;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Feeder extends Subsystem {


    private static Feeder instance = null;
    public static Feeder getInstance() {
        if(instance == null){
            instance = new Feeder();
        }
        return instance;
    }

    //all states are basically different speeds
    public enum State{
        IDLE,
        //for when we are feeding the shooter to shoot
        FEEDING,
        REVERSE,
        //for when we are intaking
        INTAKING,
        STAGING,
        FEEDINGSECOND;
    }

    //OOPSIE means you went to far
    public enum bannerState {
        EMPTY,INFEEDER,ONEBALLSTAGED,TWOBALL,READY,TRANSFER,STAGINGONE, STAGINGTWO, UNDEFINED;
    }

    //instantiate vital class varibles; motors, state etc.
    private LazyTalonFX feeder;
    private AnalogInput banner1, banner2;
    //starts in idle state
    private State currentState = State.IDLE;
    private boolean isDoneShooting = false;
    private boolean stateChanged = false;
    //assumes intake has existed since beginning of time
    private double stateChangedTimestamp = Double.POSITIVE_INFINITY;
    private boolean hasBall = false;
    private boolean[] ballPoss;
    private int ballCount;
    private bannerState prevState;
    private boolean ballStaged = false;

    public Feeder(){
        feeder = new LazyTalonFX(Ports.FEEDER, "canivore");
        //banner1 = new AnalogInput(Ports.FEEDER_BANNER1);
        //banner2 = new AnalogInput(Ports.FEEDER_BANNER2);
        banner1 = new AnalogInput(0);
        banner2 = new AnalogInput(1);
        ballPoss = new boolean[] {false,false};
        ballCount = 0;
        prevState = bannerState.EMPTY;

        feeder.setNeutralMode(NeutralMode.Coast);
        feeder.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        feeder.enableVoltageCompensation(true);
        feeder.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        feeder.enableVoltageCompensation(true);


        //the 0 is PID slot just look at the function youre calling
        feeder.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);
        
        feeder.configOpenloopRamp(Constants.Feeder.kRamp, 10);

        //limit supply current to not kill motors if a ball jams; current limit is in constants, under feeder ofc
        feeder.configSupplyCurrentLimit(Constants.Feeder.currentLimitConfig);

        feeder.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 50);
        feeder.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000);
    }

    //for other subsystems to know if we have a ball and if we've finished shooting or not
    public boolean hasBall(){return hasBall;}
    public boolean isDoneShooting(){return isDoneShooting;}

    ///////Hardware Functions////////
    /**
     * Just apply a voltage to the motors; used as a way to apply a volatge to motors through out the subsystem
     * @param output Percent voltage to send to motor
     */
    public synchronized void setMotor(double output) {
        feeder.set(ControlMode.PercentOutput, output);
    }



    public int getBallCount(){
        return ballCount;
    }
    /**
     * Just get the banner sensor's reading
     * @return
     */

    public boolean getBanner1(){
        double voltage = banner1.getVoltage();
        //System.out.println(banner1.getVoltage());
        if(voltage<4.0){
            return false;
        }else{
            return true;
        }
    }

    public boolean getBanner2(){
        double voltage = banner2.getVoltage();
        //System.out.println(banner2.getVoltage());
        if(voltage<4.0){
            return false;
        }else{
            return true;
        }
    }

    //////State///////
    /**
     * returns the intake's current state
     * @return
     */
    public State getState(){
        return currentState;
    }

    /**
     * Main function for changing the intake state; takes note of time of last state change
     * @param newState The state you want to enter
     */
    public void setState(State newState) {
        if(newState != currentState){
            //log that the state has changed and at what time the last change was
            stateChanged = true;
            stateChangedTimestamp = Timer.getFPGATimestamp();
        }
        currentState = newState;
        //shooting finishes after states are changed
        isDoneShooting = false;
    }

    /**
     * Sets the intake's new state; the speed is determined by the state
     * @param desiredState The state you want to enter
     */
    public void conformToState(State desiredState){
        setState(desiredState);
    }

    ////////Requests/////////
    /**
     * The request you send to the intake to change its state; the main way to change intake states
     * @param desiredState The state you want to enter
     * @return
     */
    public Request stateRequest(State desiredState) {
        return new Request(){

            @Override
            public void act() {
                conformToState(desiredState);
            }

        };
    }

    public Request feederStageRequest(){
        return new Request(){
            
            
            @Override
            public void act(){
                System.out.println("state was changed");
                conformToState(State.STAGING);
            }

            @Override
            public boolean isFinished() {
                if (currentState == State.IDLE){
                    return true;
                }else{
                    return false;
                }
            }


        };
    }
    

    /**
     * Request to wait until the robot has finished shooting, basically till it chages states
     * @return
     */
    public Request waitToFinishShootingRequest() {
        return new Request() {

            @Override
            public void act() {
            }

            @Override
            public boolean isFinished() {
                return isDoneShooting;
            }
        };
    }


    /**
     * This is for ejecting and unjamiming; you can send it a time to outake for to unjam
     * @param seconds How long to wait in seconds
     * @return
     */
    public Request timedReverseRequest(double seconds) {
        return new Request(){

            @Override
            public void act() {
                conformToState(State.REVERSE);
            }

            @Override
            public boolean isFinished() {
                /**
                 * your last state change was this action; it took note of the time it chnged
                 * now wait until the amount of seconds you specified until stopping the intake
                 */
                if ((Timer.getFPGATimestamp() - stateChangedTimestamp) >= seconds) {
                    conformToState(State.IDLE);
                    return true;
                }
                return false;
            }
        };
    }

    private final Loop loop = new Loop(){

        @Override
        public void onStart(double timestamp) {
            conformToState(State.IDLE);
            prevState = bannerState.EMPTY;
            ballCount = 0;
        }

        @Override
        public void onLoop(double timestamp) {
            //generally in the order in which they should happen
            //System.out.println(getBanner1() +" "+ getBanner2());
            //System.out.println(banner1.getVoltage() +" "+ banner2.getVoltage());
            //System.out.println(ballCount);
            //System.out.println(prevState);
            //System.out.println(currentState);
            switch(currentState){

                case INTAKING:
                if(getBanner1()){
                    setMotor(Constants.Feeder.kReceivingSpeed);
                    ballCount = 1;
                }else{
                    setMotor(0.0);
                }
                
                break;
                case FEEDING:
                    
                    setMotor(Constants.Feeder.kFeedingSpeed);
                    prevState = bannerState.EMPTY;

                    if (Timer.getFPGATimestamp() - stateChangedTimestamp >= Constants.Feeder.kFeedingTime){
                        conformToState(State.IDLE);
                        isDoneShooting = true;
                        ballCount = 0;
                    }

            
                    break;


                case FEEDINGSECOND:
                    setMotor(Constants.Feeder.kFeedingSpeed);
                    prevState = bannerState.EMPTY;

                    if (Timer.getFPGATimestamp() - stateChangedTimestamp >= Constants.Feeder.kFeedingTimeSecond){
                        conformToState(State.IDLE);
                        isDoneShooting = true;
                        ballCount = 0;
                    }

        
                break;

                case STAGING:
                    
                    if (!getBanner2()){
                        System.out.println("Staging");
                        setMotor(Constants.Feeder.kStagingSpeed);
                    }else{
                        conformToState(State.IDLE);
                    }

                break;
                case REVERSE:
                    //when reversing we got rid of our balls
                    setMotor(Constants.Feeder.kReverseSpeed);
                    if (stateChanged) {
                        hasBall = false;
                        ballCount = 0;
                        prevState = bannerState.EMPTY;
                    }
                    break;
                case IDLE:
                    setMotor(0.0);
                    break;
                default:
                    DriverStation.reportError("Invalid Feeder State", false);
            }
            //when you change states its in the past youve already done it now its false
            if(stateChanged) {
                stateChanged = false;
            }

        }

        @Override
        public void onStop(double timestamp) {
            conformToState(State.IDLE);
        }
    };



    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Intake Banner", getBanner1());
        SmartDashboard.putBoolean("Feeder Banner 1", getBanner2());
        SmartDashboard.putNumber("Ball Count", ballCount);
        SmartDashboard.putBoolean("Feeder Is Done Shooting", isDoneShooting);
        SmartDashboard.putString("Feeder State", currentState.toString());
        SmartDashboard.putBoolean("Do We Have a Ball", hasBall);
        SmartDashboard.putNumber("Feeder Current", feeder.getSupplyCurrent());
        if (Settings.debugFeeder()) {
            SmartDashboard.putNumber("Feeder Percent Out", feeder.getMotorOutputPercent());
            SmartDashboard.putNumber("Feeder Current", feeder.getSupplyCurrent());
            SmartDashboard.putNumber("Feeder Voltage", feeder.getMotorOutputVoltage());
        }
    }

    @Override
    public void stop() {
        conformToState(State.IDLE);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

}
