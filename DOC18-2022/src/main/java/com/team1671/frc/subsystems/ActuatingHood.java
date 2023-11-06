/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1671.frc.subsystems;

import java.util.Optional;

import com.team1671.frc.Constants;
import com.team1671.frc.Ports;
import com.team1671.frc.Settings;
import com.team1671.frc.RobotState;
import com.team1671.frc.loops.ILooper;
import com.team1671.frc.loops.LimelightProcessor;
import com.team1671.frc.loops.Loop;
import com.team1671.frc.subsystems.requests.Request;
import com.team1671.frc.vision.ShooterAimingParameters;
import com.team1671.lib.util.Limelight;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Hood that controls the trajectory of the ball
public class ActuatingHood extends Subsystem {
    //define/set variables
    Servo hoodServo1;
    Servo hoodServo2;
    Limelight limelight;

    RobotState robotState;

    private static ActuatingHood instance = null;
    public static ActuatingHood getInstance() {
        if (instance == null)
            instance = new ActuatingHood();
        return instance;
    }
    //define the servos
    public ActuatingHood() {
        hoodServo1 = new Servo(Ports.HOOD1);
        hoodServo2 = new Servo(Ports.HOOD2);
        robotState = RobotState.getInstance();
        limelight = new Limelight();
    }
    //posible states of the hood, including the angle they are at
    public enum State {
        CLOSE, FAR, NOVISION
    }
    //sets state and preserves the timestamp of swich for later use
    public State currentState = State.CLOSE;
    private double stateEnteredTimestamp = 0.0;
    public void setState(State newState) {
        if (currentState != newState) 
            stateEnteredTimestamp = Timer.getFPGATimestamp();
        currentState = newState;
    }


    public void setFar(){
        hoodServo1.set(0.8);
        hoodServo2.set(0.8);
        //sSystem.out.println("setting far");
    }

    public void setClose(){
        hoodServo1.set(0.4);
        hoodServo2.set(0.4);
        //System.out.println("setting close");
    }

    public void setNoVision(){
        hoodServo1.set(0.2);
        hoodServo2.set(0.2);
    }

    public State getState(){
        return currentState;
    }

    //request state
    public Request stateRequest(State desiredState) {
        return new Request(){
        
            @Override
            public void act() {
                setState(desiredState);
            }
        };
    }

    private final Loop loop = new Loop(){

        @Override
        public void onStart(double timestamp) {
           //setClose();
        }

        @Override
        public void onLoop(double timestamp) { 
            
            //hoodServo1.set(0.2);
            //hoodServo2.set(0.2);
            switch(currentState){
                case FAR: 
                setFar();
                break;

                case CLOSE:
                setClose();
                break;

                case NOVISION:
                setNoVision();
                break;
               
                
            }

        }

        @Override
        public void onStop(double timestamp) {
            //setClose();
            
        }

    };

    public Request visionExtensionRequest() {
        return new Request(){
        
            @Override
            public void act() {
                visionExtension();
            }
        };
        
    }

    public Request moveRequest(double signal) {
        return new Request(){
        
            @Override
            public void act() {
               hoodServo1.set(signal);
               hoodServo2.set(signal);
            }
        };
        
    }

    //extend the actuator depending on the vision
    public void visionExtension() {
                if (limelight.getTV(0.0) != 0){
                    if (limelight.getDistance() >= Constants.ActuatingHood.kFarRangeThreshold) {
                        System.out.println("HOOD STATE SET: FAR" + " VISION DISTANCE: " + limelight.getDistance());
                        setState(State.FAR);
                    } else {
                        System.out.println("HOOD STATE SET: CLOSE" + " VISION DISTANCE: " + limelight.getDistance());
                        setState(State.CLOSE);
                    }
                }else{
                    System.out.println("no valid target found yet");
                }
            }

    //telemetry
	@Override
	public void outputTelemetry() {
            SmartDashboard.putNumber("Servo Position", hoodServo1.get());
            SmartDashboard.putString("Hood State", getState().toString());
	}

	@Override
	public void stop() {
        setClose();
	}

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

}
