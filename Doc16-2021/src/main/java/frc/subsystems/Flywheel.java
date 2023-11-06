package frc.subsystems;

import java.lang.invoke.VolatileCallSite;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import drivers.LazyTalonFX;
import drivers.LazyTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.loops.Loop;
import frc.loops.Looper;
import frc.robot.Constants;
import frc.util.Limelight;

public class Flywheel extends Subsystem{
    private static Flywheel instance;
    public static Flywheel getInstance() {
        if(instance == null) {
            instance = new Flywheel();
        }
        return instance;
    }

    public class FlywheelState {
        
        public double vel = 0.0;
    }

    public enum State {
        IDLE,
        SHOOTING,
        SPIN_UP;
    }
    public State currState = State.IDLE;

    public State getState() {
        return currState;
    }

    public void setState(State newState){
        if(currState!=newState){
            currState=newState;
        }else{
            System.out.println("no");
        }
    }

    public void setWithMotionMagic(double rpm){
        leftFlywheel.set(ControlMode.MotionMagic, rpmToEncVelocity(rpm));
        //rightFlywheel.set(ControlMode.PercentOutput, -);
    }

    public TalonFX leftFlywheel, rightFlywheel;
    public TalonSRX acceleratorWheels;
    private Limelight limelight = Limelight.getInstance();

    public FlywheelState wantedState = new FlywheelState();
    public FlywheelState state = new FlywheelState();
 

    private Flywheel() {
        leftFlywheel = new LazyTalonFX(Constants.LEFT_FLYWHEEL);
        rightFlywheel = new LazyTalonFX(Constants.RIGHT_FLYWHEEL);
        rightFlywheel.set(ControlMode.Follower, Constants.LEFT_FLYWHEEL);
        rightFlywheel.setInverted(InvertType.OpposeMaster);
        //acceleratorWheels = new LazyTalonSRX(Constants.ACCELERATOR_WHEEL);

        rightFlywheel.enableVoltageCompensation(true);
        leftFlywheel.enableVoltageCompensation(true);
        //acceleratorWheels.enableVoltageCompensation(true);

        rightFlywheel.configVoltageCompSaturation(12.0, Constants.LONG_CAN_TIMEOUT_MS);
        leftFlywheel.configVoltageCompSaturation(12.0, Constants.LONG_CAN_TIMEOUT_MS);
        //acceleratorWheels.configVoltageCompSaturation(12.0, Constants.LONG_CAN_TIMEOUT_MS);

        leftFlywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.LONG_CAN_TIMEOUT_MS);


        rightFlywheel.setNeutralMode(NeutralMode.Brake);
        leftFlywheel.setNeutralMode(NeutralMode.Brake);
        //acceleratorWheels.setNeutralMode(NeutralMode.Brake);

        leftFlywheel.selectProfileSlot(0, 0);
        
        //ramp rates, when loop is open, we don't care, hence the 0
        leftFlywheel.configClosedloopRamp(1.0, 10);
        leftFlywheel.configOpenloopRamp(0.0, 10);

        //this ensures that if our error gets too high, we'll clear the integral gain
        //this is because I in PID is finnicky
        leftFlywheel.config_IntegralZone(0, (int)rpmToEncVelocity(200.0));

        //pid fun. channel 0
        leftFlywheel.config_kP(0, Constants.kShooterP);
        leftFlywheel.config_kI(0, Constants.kShooterI);
        leftFlywheel.config_kD(0, Constants.kShooterD);
        leftFlywheel.config_kF(0, Constants.kShooterF);
        
    }

    public double encVelociyToRPM(double encVelociy){
        return encVelociy / 2048 * 600. / 1.125;
    }
    public double rpmToEncVelocity(double rpm){
        return rpm * 2048 / 600. * 1.125;
    }
    public double getRPM(){
        return encVelociyToRPM(leftFlywheel.getSelectedSensorPosition());
    }
    
    private void setMotors(double velocity) {
        leftFlywheel.set(ControlMode.PercentOutput, velocity);
        //rightFlywheel.set(ControlMode.PercentOutput, -velocity);
        //acceleratorWheels.set(ControlMode.PercentOutput, -velocity);
    }

    public synchronized void setWantedState(FlywheelState wanted) {
        wantedState = wanted;
    }

    public double calculateFlywheelVel() {
        double d = limelight.getDistance();
        double vel = 0;
        if(d >= 75 && d <= 150){
            vel = 0.72;
        }else{
            vel = 0.0000000075542*(d*d*d*d) - 0.0000046959943*(d*d*d) + 0.0010717550045*(d*d) - 0.1046443640881*d + 4.3682218413152;
        }
        return vel;
    }

    public void FlywheelHandler(FlywheelState wanted, FlywheelState current, double timestamp) {
        setMotors(wanted.vel);
    }

    public double flywheelVel(){
        return state.vel;
    }

    Loop loop = new Loop(){
    
        @Override
        public void onStop(double timestamp) {
            stop();            
        }
    
        @Override
        public void onStart(double timestamp) {
            zeroSensors();
            //wantedState.vel = 0;
        }
    
        @Override
        public void onLoop(double timestamp) {
            synchronized(Flywheel.this){
                //FlywheelHandler(wantedState, state, timestamp);
                switch(currState){
                    case IDLE:
                        setMotors(0.0);
                        break;
                    case SHOOTING:
                        //failsafe
                        setMotors(.6666666666667);
                        break;
                    case SPIN_UP:
                        setWithMotionMagic(4253.333333333333333);
                        break;
                    default: 
                        break;
                }
            }            
        }
    };

    public void writeToLog() {
    }

    public void outputToSmartDashboard(){
        SmartDashboard.putNumber("Left Flywheel Voltage", leftFlywheel.getMotorOutputVoltage());
        SmartDashboard.putNumber("Right Flywheel Voltage", rightFlywheel.getMotorOutputVoltage());
        SmartDashboard.putNumber("Accelerator Wheel Voltage", acceleratorWheels.getMotorOutputVoltage());
        SmartDashboard.putNumber("Accelerator Supply Amps", acceleratorWheels.getSupplyCurrent());
        SmartDashboard.putNumber("Accelerator Stator Amps", acceleratorWheels.getStatorCurrent());
    }
    public void stop(){
        setMotors(0.0);
    }
    public void zeroSensors(){
    }
    public void registerEnabledLoops(Looper enabledLooper){
        enabledLooper.register(loop);
    }
}