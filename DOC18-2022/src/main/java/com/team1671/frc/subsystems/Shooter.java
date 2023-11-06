/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1671.frc.subsystems;

import java.lang.annotation.Target;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1671.frc.Constants;
import com.team1671.frc.Ports;
import com.team1671.frc.RobotState;
import com.team1671.frc.Settings;
import com.team1671.frc.loops.ILooper;
import com.team1671.frc.loops.LimelightProcessor;
import com.team1671.frc.loops.Loop;
import com.team1671.frc.subsystems.requests.Request;
import com.team1671.frc.vision.ShooterAimingParameters;
import com.team1671.lib.util.CircularBuffer;
import com.team1671.lib.util.InterpolatingDouble;
import com.team1671.lib.util.Limelight;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
    
    LazyTalonFX shooterMaster, shooterSlave;
    
    RobotState robotState;
    Limelight limelight;

    PeriodicIO periodicIO = new PeriodicIO();

    private static Shooter instance = null;
    public static Shooter getInstance() {
        if (instance == null) 
        instance = new Shooter();
        return instance;
    }
    
    public enum controlState {
        OPEN_LOOP, SPIN_UP, HOLD_WHEN_READY, HOLD
    }
    
   
    
    private controlState currentState = controlState.OPEN_LOOP;
    
    
    public controlState getTopState() {
        return currentState;
    }
    
    double targetRPM = 0.0;
    double lastTopRPM = 0.0;
    double lastBottomRPM = 0.0;
    double onTargetStartTime = Double.POSITIVE_INFINITY;
    double bottomOnTargetStartTime = Double.POSITIVE_INFINITY;

    public boolean isBackUpToSpeed = false;
    boolean onTarget = false;
    boolean bottomOnTarget = false;
    
    private Shooter() {
        limelight = new Limelight();
        shooterMaster = new LazyTalonFX(Ports.MASTER_FLYWHEEL, "canivore");
        shooterSlave = new LazyTalonFX(Ports.SLAVE_FLYWHEEL, "canivore");

        robotState = RobotState.getInstance();
        
        
        shooterMaster.configNominalOutputReverse(0.0, Constants.kLongCANTimeoutMs);
        shooterMaster.configPeakOutputReverse(0.0, Constants.kLongCANTimeoutMs);
            
        shooterSlave.setInverted(TalonFXInvertType.OpposeMaster);
        shooterSlave.set(TalonFXControlMode.Follower, Ports.MASTER_FLYWHEEL);
            
        shooterMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        shooterMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kLongCANTimeoutMs);
        shooterSlave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        shooterSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kLongCANTimeoutMs);

        shooterMaster.selectProfileSlot(0, 0);
            

        shooterMaster.setNeutralMode(NeutralMode.Coast);
        shooterSlave.setNeutralMode(NeutralMode.Coast);
        shooterMaster.configClosedloopRamp(Constants.Shooter.kShooterRamp, 10);
        shooterMaster.configOpenloopRamp(0.0, 10);

        shooterMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 40.0, 60.0, 1000.0));
        shooterSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 40.0, 60.0, 1000.0));


        shooterMaster.config_IntegralZone(0, (int)rpmToEncVelocity(200.0));
        shooterSlave.config_IntegralZone(0, (int)rpmToEncVelocity(200.0));
        
        shooterMaster.config_kP(0, Constants.Shooter.kP);
        shooterMaster.config_kI(0, Constants.Shooter.kI);
        shooterMaster.config_kD(0, Constants.Shooter.kD);
        shooterMaster.config_kF(0, Constants.Shooter.kF);
        
        shooterMaster.config_kP(1, Constants.Shooter.kHoldP);
        shooterMaster.config_kI(1, Constants.Shooter.kHoldI);
        shooterMaster.config_kD(1, Constants.Shooter.kHoldD);
        shooterMaster.config_kF(1, Constants.Shooter.kHoldF);
        
        
    }

    private int bufferWindowSize = 25; //50
    private CircularBuffer mTopKfEstimator = new CircularBuffer(bufferWindowSize);

    private double estimateKf(double rpm, double percentOutput) {
        final double speed_in_ticks_per_100ms = rpmToEncVelocity(rpm);
        final double output = 1023.0 * percentOutput; //1023 is the magic number: don't question the tech.
        return output / speed_in_ticks_per_100ms;
    }

    private void resetHold() {
        mTopKfEstimator.clear();
    }
  
      


    public boolean isOpenLoop() {
        return currentState == controlState.OPEN_LOOP;
    }

    private void configForSpinup() { 
           
            currentState = controlState.SPIN_UP;
            shooterMaster.selectProfileSlot(0, 0);
            shooterMaster.enableVoltageCompensation(true);
            shooterSlave.enableVoltageCompensation(true);
            shooterMaster.configClosedloopRamp(Constants.Shooter.kShooterRamp);
        }

    private void configForHold() {
        currentState = controlState.HOLD;
        
        shooterMaster.selectProfileSlot(1, 0);
        shooterMaster.configClosedloopRamp(0.0);
        shooterMaster.enableVoltageCompensation(false);
        shooterSlave.enableVoltageCompensation(false);

        shooterMaster.config_kF(1, mTopKfEstimator.getAverage());
        shooterMaster.set(ControlMode.Velocity, rpmToEncVelocity(targetRPM));
        //System.out.println("Top Kf Estimate: " + mTopKfEstimator.getAverage());
        //System.out.println("Bottom kf Estimate: " + mBottomKfEstimator.getAverage());
    }

    private void configForHoldWhenReady() {
        currentState = controlState.HOLD_WHEN_READY;
        shooterMaster.selectProfileSlot(0, 0);
        shooterMaster.enableVoltageCompensation(true);
        shooterSlave.enableVoltageCompensation(true);
        shooterMaster.configClosedloopRamp(Constants.Shooter.kShooterRamp);
        shooterMaster.set(ControlMode.Velocity, estimatePercentOut(targetRPM));
    }

    private synchronized void handleClosedLoop(double timestamp) {
        final double currentTopSpeed = getRPM();

        switch (currentState) {
            case SPIN_UP:
                shooterMaster.set(ControlMode.Velocity, rpmToEncVelocity(targetRPM));
                resetHold();
                break;
            case HOLD_WHEN_READY:
                final double abs_error = Math.abs(currentTopSpeed - targetRPM);
                final boolean on_target_now = onTarget ? abs_error < 50 : abs_error < 25; // 75 | 25
                if (on_target_now && !onTarget) {
                    // First cycle on target.
                    onTargetStartTime = timestamp;
                    onTarget = true;
                } else if (!on_target_now) {
                    resetHold();
                }

                if (onTarget) {
                    // Update Kf.
                    mTopKfEstimator.addValue(estimateKf(currentTopSpeed, shooterMaster.getMotorOutputPercent()));
                }
                break;

            case HOLD:
                if (Math.abs(getRPM() - targetRPM) > 50){
                    isBackUpToSpeed = false;
                }else{
                    isBackUpToSpeed = true;
                }
            break;
            default:
                break;
        }


        if (mTopKfEstimator.getNumValues() >= bufferWindowSize) {
            configForHold();
        } else {
            shooterMaster.set(ControlMode.Velocity, rpmToEncVelocity(targetRPM));
        }
    }

    public synchronized void spinUpShooters(double RPMSetpoint) {
        setSpinUp(RPMSetpoint);
    }

    public synchronized void setSpinUp(double rpmSetpoint) {
        
        if (currentState != controlState.SPIN_UP){
            configForSpinup();
            targetRPM = rpmSetpoint;
        }
    }

    public synchronized void setHoldWhenReady(double RPMSetpoint) {
        targetRPM = RPMSetpoint;
        configForHoldWhenReady();
    }

    public synchronized void setOpenLoop(double demand) {
        currentState = controlState.OPEN_LOOP;
        shooterMaster.set(ControlMode.PercentOutput, demand);
    }


    public double getRPMSetpoint() {
        if (currentState != controlState.OPEN_LOOP)
            return encVelocityToRPM(shooterMaster.getClosedLoopTarget());
        return 0.0;
    }

    


    public synchronized boolean hasReachedSetpoint() {
        return (shooterMaster.getControlMode() == ControlMode.Velocity && currentState == controlState.HOLD);
    }

    private synchronized double estimatePercentOut (double rpm) {
            return rpmToEncVelocity(rpm); //* Constants.kWheelTreeMap.getInterpolated(new InterpolatingDouble(rpm)).value / 1023.0;
    }

    private synchronized double getRPM() {
        return encVelocityToRPM(periodicIO.velocity);
    }


    public double encVelocityToRPM(double encVelocity) {
        //falcon encoder tick rate = 2048 ticks/s           CANcoders + MAGencoders = 4096 ticks/s        
        return encVelocity / 2048.0 * 600.0 / Constants.Shooter.kShooterEncToOutputRatio;
    }

    public double rpmToEncVelocity(double RPM) {
        return RPM * 2048.0 / 600.0 * Constants.Shooter.kShooterEncToOutputRatio;
    }

    public Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            mTopKfEstimator.clear();
        }

        @Override
        public void onLoop(double timestamp) {
            //System.out.println("limelight target exists" + limelight.getTV(0.0));
            //System.out.println("limelight distance from target" + limelight.getDistance());
            synchronized (Shooter.this) {
                if (currentState != controlState.OPEN_LOOP) {
                    handleClosedLoop(timestamp);
                } else {
                    mTopKfEstimator.clear();
                    onTargetStartTime = Double.POSITIVE_INFINITY;
                    onTarget = false;
                    bottomOnTargetStartTime = Double.POSITIVE_INFINITY;
                    bottomOnTarget = false;
                }

            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }

    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.velocity = shooterMaster.getSelectedSensorVelocity(0);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Shooter RPM", getRPM());
        SmartDashboard.putBoolean("Shooter On Target", hasReachedSetpoint());
        SmartDashboard.putString("Shooter Top State", getTopState().toString());
        SmartDashboard.putNumber("Shooter Top Current", shooterMaster.getOutputCurrent());
        SmartDashboard.putNumber("Shooter Top Percent Output", shooterMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter Top Voltage", shooterMaster.getMotorOutputVoltage());
        
        if (Settings.debugShooter()) {
            SmartDashboard.putNumber("Shooter Top Setpoint RPM", getRPMSetpoint());
            SmartDashboard.putNumber("Shooter Top Voltage", shooterMaster.getMotorOutputVoltage());
            SmartDashboard.putString("Shooter Top State", getTopState().toString());
        }
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }



    public Request openLoopRequest(double output) {
        return new Request() {

            @Override
            public void act() {
                setOpenLoop(output);
            }


        };
    }

    public Request waitToHoldRequest(){
        return new Request(){

            @Override
            public void act(){

            }

            @Override
            public boolean isFinished(){
                return isBackUpToSpeed;
            }
        };
    }

    public Request holdWhenReadyRequestRequest(double RPM) {
        return new Request(){
        
            @Override
            public void act() {
                setHoldWhenReady(RPM);
            }

            
            public boolean isFinished() {
                return hasReachedSetpoint();
            }
        };
    }

    public Request spinUpRequest(double RPM) {
        return new Request(){
        
            @Override
            public void act() {
                spinUpShooters(RPM);
            }
        };
    }

    public Request holdWhenReadyRequest(double distanceToTarget) {
        return new Request(){
        
            @Override
            public void act() {
                double wheelRPM = Constants.kWheelRPMToDistanceTreeMap.getInterpolated(new InterpolatingDouble(distanceToTarget)).value;
                setHoldWhenReady(wheelRPM);
            }


        };
    }

    public Request visionHoldWhenReadyRequest() {
        return new Request(){
        
            @Override
            public void act() {
                double distFromTarget = limelight.getDistance();
                if (limelight.getTV(0.0) != 0) {
                    System.out.println("Vision Target - " + distFromTarget);
                    double wheelRPM = Constants.kWheelRPMToDistanceTreeMap.getInterpolated(new InterpolatingDouble(distFromTarget)).value;
                    setHoldWhenReady(wheelRPM);
                } else {
                    System.out.println("No Vision Target Found");
                    setHoldWhenReady(3100);
                }
            }

            @Override
            public boolean isFinished() {
                return hasReachedSetpoint();
            }

        };
    }    

    public class PeriodicIO {
        public double velocity = 0.0;
    }
    
}