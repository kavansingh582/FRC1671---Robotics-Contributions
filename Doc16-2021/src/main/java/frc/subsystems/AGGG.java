package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import drivers.LazyTalonFX;
import frc.loops.Loop;
import frc.loops.Looper;
import frc.robot.Constants;

public class AGGG extends Subsystem{
    private static AGGG instance;

    public static AGGG getInstance() {
        if(instance == null) {
            instance = new AGGG();
        }
        return instance;
    }

    public enum AGGGState{
        OPEN_LOOP,
        EXTENDING
    }

    private TalonFX left, right;
    public AGGGState state = AGGGState.OPEN_LOOP;
    public boolean isRightBrakeMode, isLeftBrakeMode;


    private AGGG() {
        left = new LazyTalonFX(Constants.AGGG_LEFT);
        right = new LazyTalonFX(Constants.AGGG_RIGHT);

        right.enableVoltageCompensation(true);
        right.configVoltageCompSaturation(12.0, Constants.LONG_CAN_TIMEOUT_MS);
        right.configOpenloopRamp(0.2, Constants.LONG_CAN_TIMEOUT_MS);

        left.enableVoltageCompensation(true);
        left.configVoltageCompSaturation(12.0, Constants.LONG_CAN_TIMEOUT_MS);
        left.configOpenloopRamp(0.2, Constants.LONG_CAN_TIMEOUT_MS);

        left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.LONG_CAN_TIMEOUT_MS);
        right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.LONG_CAN_TIMEOUT_MS);
        left.setSelectedSensorPosition(0);
        right.setSelectedSensorPosition(0);

        isRightBrakeMode = false;
        isLeftBrakeMode = false;
    }

    public void setMotors(double leftPower, double rightPower){
        if(getLeftEncoder() < 0 && leftPower < 0){
            leftPower = 0;
        }else if(getLeftEncoder() > 71 && leftPower > 0){
            leftPower = 0;
        }else{
            leftPower = leftPower;
        }

        if(getRigthEncoder() < 0 && rightPower < 0){
            rightPower = 0;
        }else if(getRigthEncoder() > 71 && rightPower > 0){
            rightPower = 0;
        }else{
            rightPower= rightPower;
        }

        right.set(ControlMode.PercentOutput, rightPower);
        left.set(ControlMode.PercentOutput, rightPower);
    }

    public synchronized void setState(AGGGState wantedState, AGGGState currentState){
        if(currentState != wantedState){
            currentState = wantedState;
        }
    }

    public synchronized void setRightBrakeMode(boolean on) {
        if(isRightBrakeMode != on) {
            isRightBrakeMode = on;
            if(on) {
                right.setNeutralMode(NeutralMode.Brake);
            } else {
                right.setNeutralMode(NeutralMode.Coast);
            }
        }
    }

    public synchronized void setLeftBrakeMode(boolean on) {
        if(isLeftBrakeMode != on) {
            isLeftBrakeMode = on;
            if(on) {
                left.setNeutralMode(NeutralMode.Brake);
            } else {
                left.setNeutralMode(NeutralMode.Coast);
            }
        }
    }

    public synchronized void setOpenLoop(double left, double right) {
        if(state != AGGGState.OPEN_LOOP) {
            state = AGGGState.OPEN_LOOP;
            setLeftBrakeMode(false);
            setRightBrakeMode(false);
        }
        setMotors(left, right);
    }

    public synchronized void handleExtend(){

    }

    public double getLeftEncoder(){ //4 in per rev of lead screw
        return (left.getSelectedSensorPosition() / Constants.FALCON_PULSE_PER_REV);
    }
    public double getRigthEncoder(){
        return (right.getSelectedSensorPosition() / Constants.FALCON_PULSE_PER_REV);
    }
    public void zeroEncoders(){
        right.setSelectedSensorPosition(0);
        left.setSelectedSensorPosition(0);
    }

    Loop loop = new Loop(){
    
        @Override
        public void onStop(double timestamp) {
            stop();
        }
    
        @Override
        public void onStart(double timestamp) {
            zeroSensors();
        }
    
        @Override
        public void onLoop(double timestamp) {
            synchronized(AGGG.this) {
                switch(state){
                    case OPEN_LOOP:
                        break;
                    case EXTENDING:
                        handleExtend();
                        break;
                    default: System.out.println("Unexpected AGGG state: " + state);
                }
            }
        }
    };
    
    public void writeToLog() {
    };
    public void outputToSmartDashboard() {
    }
    public void stop() {
    }
    public void zeroSensors() {
        zeroEncoders();
    }
    public void registerEnabledLoops(Looper enabledLooper){
        enabledLooper.register(loop);
    }
}