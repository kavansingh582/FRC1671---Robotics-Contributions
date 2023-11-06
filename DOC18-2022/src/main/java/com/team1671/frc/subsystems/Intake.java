package com.team1671.frc.subsystems;

//imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.team1671.frc.Constants;
import com.team1671.frc.Ports;
import com.team1671.frc.Settings;
import com.team1671.frc.loops.ILooper;
import com.team1671.frc.loops.Loop;
import com.team1671.frc.subsystems.requests.Request;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Intake extends Subsystem {
    
    // create an instance of the intake
    public static Intake instance = null;
    public static Intake getInstance(){
        if (instance == null){
            instance = new Intake();
        }
        return instance;
    }

    //all possible intake states, like intaking vs ejecting
    public enum State{
        IDLE(0.0, false),
        INTAKING(Constants.Intake.kIntakeSpeed, true),
        EJECTING(Constants.Intake.kOuttakeSpeed, true),
        FEEDING(Constants.Intake.kFeedingSpeed, true);
    
        double speed;
        boolean isExtended;
        State(double speed, boolean isExtended){
            this.speed = speed;
            this.isExtended = isExtended;
        }
    }

    //creating motors and states
    private LazyTalonFX intakeMotor;
    private Solenoid extender;
    private State currentState = State.IDLE;
    private boolean isExtended = false;


    //defining the properties of motors
    public Intake(){
        intakeMotor = new LazyTalonFX(Ports.INTAKE_MOTOR, "canivore");
        extender = new Solenoid(Ports.PCM, PneumaticsModuleType.CTREPCM, Ports.INTAKE_EXTENDER);

        intakeMotor.setInverted(false);
        
        intakeMotor.configOpenloopRamp(0.25);

        //limit CAN usage
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 50);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000);
    }

     /**
     * Opens or closes the extender
     * @param extend To extend or not to extend
     */
    public void fireExtender(boolean extend){
        //if already extended or retracted dont do it again
        if(isExtended != extend){
            isExtended = extend;
        }
        //set enxtender value
        extender.set(isExtended);
    }

    /**
     * Sets the intake roller to a percent from -1 to 1
     * @param percentVoltage Percent power to apply; -1 to 1
     */
    public void setOpenLoop(double percentVoltage){
        intakeMotor.set(ControlMode.PercentOutput, percentVoltage);
    }

    /**
     * Sets current intake state to the state you pass
     * @param newState The state you want; comes from the enum of states
     */
    public void setState(State newState){
        if(currentState != newState){
            currentState = newState;
        }else{
        
        }
    }

    /**
     * Changes the state of the intake to desiredState using setState()
     * @param desiredState The state you want the intake to be in
     * @param intakeOverrideVoltage A voltage to apply to the intake
     */
    public void conformToState(State desiredState, double intakeOverrideVoltage){
        setState(desiredState);
        //this could change depending on motor direction lmfao.
        setOpenLoop(intakeOverrideVoltage);
        fireExtender(desiredState.isExtended);
    }

      /**
     * Changes the state of the intake 
     * @param desiredState The state you want the intake to be in
     */
    public void conformToState(State desiredState){
        conformToState(desiredState, desiredState.speed);
    }

     /**
     * Returns the intake's current state
     */
    public State getstate(){
        return currentState;
    }


     /**
     * Changes your intake state 
     * @param desiredState
     * @return A request
     */
    public Request stateChangeRequest(State desiredState){
        return new Request(){

            @Override
            public void act(){
                conformToState(desiredState);
                System.out.println("state change for intake");
            }
        };
    }
   
    private Loop loop = new Loop() {

        //what you want to do on robot startup
        @Override
        public void onStart(double timestamp) {
            conformToState(State.IDLE);
        }

        @Override
        public void onLoop(double timestamp) {

        }
        
        //what you want to do on robot end
        @Override
        public void onStop(double timestamp) {
            conformToState(State.IDLE);
        }    
    };

    @Override
    public void outputTelemetry() {
        if(Settings.debugFeeder()){
            SmartDashboard.putNumber("Intake Motor Commanded Voltage", intakeMotor.getMotorOutputPercent());
            SmartDashboard.putNumber("Intake Motor Voltage", intakeMotor.getMotorOutputVoltage());
            SmartDashboard.putNumber("Intake Motor Amperage", intakeMotor.getSupplyCurrent());
        }
        SmartDashboard.putString("Intake State", currentState.toString());
    }

    @Override
    public void stop() {
        conformToState(State.IDLE);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper){
        enabledLooper.register(loop);
    }

}

