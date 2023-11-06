/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1671.frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team1671.frc.Constants;
import com.team1671.frc.Ports;
import com.team1671.frc.Settings;
import com.team1671.frc.loops.ILooper;
import com.team1671.frc.loops.Loop;
import com.team1671.frc.subsystems.requests.Request;
import com.team1671.lib.util.Util;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Climber class. For Traversal
 */
public class Climber extends Subsystem {

    //defining motors and solenoids needed
    LazyTalonFX hanger, hangerSlave;
    Solenoid extender, miniExtender;


    //create Climber instanec
    private static Climber instance = null;
    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    //place to store variables
    PeriodicIO periodicIO = new PeriodicIO();

    //different states to be in.
    public enum State {
        OPEN_LOOP, LOCKED, POSITION, NEUTRAL;
    }
    private State currentState = State.NEUTRAL;
    public State getState() {
        return currentState;
    }

    //variables needed.
    double targetHeight = 0.0;
    boolean onTarget = false;
    double startTime = Double.POSITIVE_INFINITY;

    //configure to a new state for loop handling.
    public void setState(State newState) {
        currentState = newState;
    }

    boolean isConfiguredForAscent = false;

    boolean isExtended = false;//false
    boolean isBrakeEngaged = true;
    //return solenoid
    public boolean isExtended() {
        return isExtended;
    }
    //return whether we are in brake mode.
    public boolean isBrakeEngaged() {
        return isBrakeEngaged;
    }


    boolean isEncoderFlipped = false;

    //whatever we want our manualSpeed to be.
    double manualSpeed = 0.5;
    public void setManualSpeed(double manualSpeed) {
        this.manualSpeed = manualSpeed;
    }

    //instantiating everything
    private Climber() {

        hanger = new LazyTalonFX(Ports.CLIMBER_MASTER, "canivore");
        hangerSlave = new LazyTalonFX(Ports.CLIMBER_SLAVE, "canivore");
        extender = new Solenoid(Ports.PCM, PneumaticsModuleType.CTREPCM, Ports.CLIMBER_EXTENDER);
        miniExtender = new Solenoid(Ports.PCM, PneumaticsModuleType.CTREPCM, Ports.MINICLIMBERPISTON);
       
        hanger.setInverted(InvertType.InvertMotorOutput);
        setEncoderPhase(false);
        
        hanger.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        hanger.enableVoltageCompensation(true);

        hanger.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);

        hangerSlave.setInverted(InvertType.OpposeMaster);
        hangerSlave.set(ControlMode.Follower, Ports.CLIMBER_MASTER);
        hanger.setInverted(false);

        hanger.setNeutralMode(NeutralMode.Brake);

        hanger.configReverseSoftLimitThreshold(0, Constants.kCANTimeoutMs);
        hanger.configForwardSoftLimitThreshold(inchesToEncUnits(Constants.Climber.kMaxControlHeight), Constants.kCANTimeoutMs);
        hanger.configForwardSoftLimitEnable(true);
        hanger.configReverseSoftLimitEnable(true);
        enableLimits(true);

        setCurrentLimit(20);  

        hanger.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        hanger.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 50);
        
        zeroSensors();
        configForAscent();
    }

    //send us into climb mode
    public void configForAscent() {
        manualSpeed = Constants.Climber.kManualSpeed;
        
        hanger.config_kP(0, Constants.Climber.kP, Constants.kCANTimeoutMs);
        hanger.config_kI(0, Constants.Climber.kI, Constants.kCANTimeoutMs);
        hanger.config_kD(0, Constants.Climber.kD, Constants.kCANTimeoutMs);
        hanger.config_kF(0, Constants.Climber.kF, Constants.kCANTimeoutMs);

        hanger.config_kP(1, Constants.Climber.kP, Constants.kCANTimeoutMs);
        hanger.config_kI(1, Constants.Climber.kI, Constants.kCANTimeoutMs);
        hanger.config_kD(1, Constants.Climber.kD, Constants.kCANTimeoutMs);
        hanger.config_kF(1, Constants.Climber.kF, Constants.kCANTimeoutMs);

        hanger.configMotionCruiseVelocity((int)(Constants.Climber.kMaxSpeed), Constants.kCANTimeoutMs);
        hanger.configMotionAcceleration((int)(Constants.Climber.kMaxSpeed * 3.0), Constants.kCANTimeoutMs);
        hanger.configMotionSCurveStrength(0);
     
        isConfiguredForAscent = true;
    }

    //set us into bring it back down
    public void configForDescent() {
        manualSpeed = 0.5;

        hanger.configMotionCruiseVelocity((int)(Constants.Climber.kMaxDescendSpeed), Constants.kCANTimeoutMs);
        hanger.configMotionAcceleration((int)(Constants.Climber.kMaxDescendSpeed * 3.0), Constants.kCANTimeoutMs);
        hanger.configMotionSCurveStrength(4);

        isConfiguredForAscent = false;
    }

    //amperage limit
    public void setCurrentLimit(int amps) {
        SupplyCurrentLimitConfiguration currentLimitConfig = new SupplyCurrentLimitConfiguration(true, amps, amps, 10);
        hanger.configSupplyCurrentLimit(currentLimitConfig);
        hangerSlave.configSupplyCurrentLimit(currentLimitConfig);
    }

    //whether we want to ignore the limits we set.
    boolean limitsEnabled = true;
    public void enableLimits(boolean enable) {
        hanger.overrideSoftLimitsEnable(enable);
        limitsEnabled = enable;
    }

    //set whether the encoder should be flipped or not.
    private void setEncoderPhase(boolean phase) {
        isEncoderFlipped = phase;
    }


    //give us what our height is
    public double getHeight() {
        return encUnitsToInches(periodicIO.position);
    }

    //give us what our velocity is.
    public double getVelocityInchesPerSecond() {
        return encUnitsToInches(periodicIO.velocity) * 10.0;
    }

    //straightforward
    private double inchesToEncUnits(double inches) {
        return (inches / Constants.Climber.kRotationToInches) * 2048;
    }

    //straightforward
    private double encUnitsToInches(double encUnits) {
        return (encUnits / 2048) * Constants.Climber.kRotationToInches;
    }

    //also straightforward
    private double hangerHeightToEncUnits(double hangerHeight) {
        return inchesToEncUnits(hangerHeight);
    }

    //rawdog command
    public void setOpenLoop(double output) {
        setState(State.OPEN_LOOP);
        periodicIO.demand = Util.scaledDeadband(output, 0.5, 0.25) * manualSpeed;
    }

    public boolean isOpenLoop() {
        return getState() == State.OPEN_LOOP;
    }


    //set the solenoids
    public void fireExtender(boolean isFired) {
        isExtended = isFired;
        extender.set(isFired);
        System.out.println("Hanger Extender Fired");
    }

    public void fireMiniExtender(boolean isFired){
        extender.set(isFired);
        System.out.println("Hanger Mini Extender Fired");
    }

    //set what target we want to be at
    public synchronized void setTargetHeight(double heightInches) {
        setState(State.POSITION);
        heightInches = Util.limit(heightInches, Constants.Climber.kMinControlHeight, Constants.Climber.kMaxControlHeight);
        if (heightInches > getHeight()) {
            hanger.selectProfileSlot(0, 0);
            configForAscent();
            System.out.println("greater");
        } else {
            System.out.println("less");
            hanger.selectProfileSlot(1, 0);
            configForDescent();
        }
        targetHeight = heightInches;
        periodicIO.demand = hangerHeightToEncUnits(heightInches);
        System.out.println("Set hanger height to: " + heightInches);
        onTarget = false;
        startTime = Timer.getFPGATimestamp();
    }

    //lock us in place
    public synchronized void lockHeight() {
        setState(State.LOCKED);
        targetHeight = getHeight();
        if(Math.abs(getHeight()) <= targetHeight){
            periodicIO.demand = periodicIO.position;
        }else{
            hanger.setNeutralMode(NeutralMode.Brake);
        }
    }

    //checks whether we are attarget height or not.
    public boolean hasReachedTargetHeight() {
        if (hanger.getControlMode() == ControlMode.MotionMagic) {
            if ((Math.abs(targetHeight - getHeight())) <= Constants.Climber.kHeightTolerance) {
                if (!onTarget) {
                    onTarget = true;
                }
                return true;
            }
        }
        return false;
    }

    private boolean areMotorsGonnaDie(double amps){
        return (hanger.getOutputCurrent() > amps || hangerSlave.getOutputCurrent() > amps);
        
    }


    //this loop just checks whether we at a dangerous current limit or not.
    private final Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            //hanger.setNeutralMode(NeutralMode.Brake);
            //pistonMove(true);
        }

        @Override
        public void onLoop(double timestamp) {
            if(areMotorsGonnaDie(15)){
                System.out.println("Climber motors are gonna die");
                //stop();
            }
        }

        @Override
        public void onStop(double timestamp) {
            //hanger.setNeutralMode(NeutralMode.Coast);
        }

    };

    //set an open loop request
    public Request openLoopRequest(double output) {
        return new Request(){
        
            @Override
            public void act() {
                setOpenLoop(output);
            }
        };
    }

    //set us to the heigt we want to be at
    public Request setHeightRequest(double height, double speedScalar) {
        return new Request() {

            @Override
            public void act() {
                setTargetHeight(height);
            } 

            @Override
            public boolean isFinished() {
                return hasReachedTargetHeight() || isOpenLoop();
            }

        };
    }

    //lock our height when we get there.
    public Request lockHeightRequest() {
        return new Request(){
        
            @Override
            public void act() {
                lockHeight();
            }
        };
    }

    public Request pistonMove(boolean extend){
        return new Request(){

            @Override
            public void act(){
                fireExtender(extend);
            }
        };
    }
    public Request fireMiniExtenderRequest(boolean extend){
        return new Request(){

            @Override
            public void act(){
                fireMiniExtender(extend);
            }
        };
    }

    
    //reads the position of the climber.
    @Override
    public void readPeriodicInputs() {
        periodicIO.position = hanger.getSelectedSensorPosition(0);
        periodicIO.velocity = hanger.getSelectedSensorVelocity(0);
        periodicIO.voltage = hanger.getMotorOutputVoltage();
        periodicIO.current = hanger.getOutputCurrent();
    }


    //set our output at all times
    @Override
    public void writePeriodicOutputs() {
        if (getState() == State.POSITION || getState() == State.LOCKED) {
            hanger.set(ControlMode.MotionMagic, periodicIO.demand);
            //System.out.println("commanding motor");
        } else {
            hanger.set(ControlMode.PercentOutput, periodicIO.demand);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    //reset the encoders
    public synchronized void zeroSensors() {
        //hanger.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        hanger.setSelectedSensorPosition(0);
    }


    //smart dashboard
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Hanger Height", getHeight());
        SmartDashboard.putNumber("Hanger Error", targetHeight - getHeight());
        SmartDashboard.putNumber("Hanger Encoder", periodicIO.position);
        SmartDashboard.putNumber("Hanger Target Height", targetHeight);
        SmartDashboard.putNumber("Hanger Demand", periodicIO.demand);


        if (Settings.debugHanger()) {
            SmartDashboard.putNumber("Hanger Raw Encoder", hanger.getSelectedSensorPosition(0));
            SmartDashboard.putNumber("Hanger Current", periodicIO.current);
            SmartDashboard.putNumber("Hanger Voltage", periodicIO.voltage);
            SmartDashboard.putNumber("Hanger Velocity", periodicIO.velocity);
            SmartDashboard.putString("Hanger State", currentState.toString());
            if (hanger.getControlMode() == ControlMode.MotionMagic) {
                SmartDashboard.putNumber("Hanger Setpoint", hanger.getClosedLoopTarget(0));
            }
        }
    }

    //stop everything
    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    public static class PeriodicIO {
        // Inputs
        public double position = 0.0;
        public double velocity = 0.0;
        public double voltage = 0.0;
        public double current = 0.0;

        // Outputs
        public double demand = 0.0;   
    }

}
