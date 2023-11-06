package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import drivers.LazyTalonSRX;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.loops.Loop;
import frc.loops.Looper;
import frc.robot.Constants;
import frc.util.Limelight;
import frc.util.PID;

public class Turntable extends Subsystem{
    private static Turntable instance;
    public static Turntable getInstance() {
        if(instance == null) {
            instance = new Turntable();
        }
        return instance;
    }

    public enum TurntableState{
        OPEN_LOOP,
        TARGETING,
        LIMELIGHT_TARGETING
    }
    public PID limelightPid = new PID(Constants.TURNTABLE_KP,Constants.TURNTABLE_KI,Constants.TURNTABLE_KD,Constants.MIN_OUTPUT,Constants.MAX_OUTPUT);
    public TalonSRX turntableMotor;
    private TurntableState turntableState = TurntableState.OPEN_LOOP;
    private boolean isBrakeMode;
    private Limelight limelight = Limelight.getInstance();

    private double turntableSetPoint;

    private Turntable() {
        turntableMotor = new LazyTalonSRX(Constants.TURNTABLE_MOTOR);

        turntableMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        turntableMotor.setSelectedSensorPosition(0);
        turntableMotor.enableVoltageCompensation(true);
        turntableMotor.configVoltageCompSaturation(10.0, Constants.LONG_CAN_TIMEOUT_MS);
        turntableMotor.configOpenloopRamp(0.0, Constants.LONG_CAN_TIMEOUT_MS);

    }

    public void setMotor(double signal){
        turntableMotor.set(ControlMode.PercentOutput, signal);
    }

    public synchronized void startTargetting(double setpoint){
        if(turntableState != TurntableState.TARGETING){
            turntableState = TurntableState.TARGETING;
        }
        turntableSetPoint = setpoint;
    }

    public synchronized void handleTargetting(double setPoint){
        double val = limelightPid.calculate(setPoint, getTurntableEncoder(), Constants.LOOPER_DT);
        setMotor(val);
    }

    public synchronized void handleLimelightTargeting() {
        double signal = 0.;
        //if detecting a target
        if(limelight.getTV(0.0) == 1){
            if(Math.abs(limelight.getTX(0.0)) > 0.05){
                signal = limelightPid.calculate(limelight.getTX(0.0), 0.0, Constants.LOOPER_DT);
            }else{
                signal = 0.0;
            }
        }else{
            signal = 0.0;
        }
        setMotor(-signal);
        System.out.println(limelight.getTX(0.0));
    }

    public synchronized void setState(TurntableState wantedTurntableState){
        turntableState = wantedTurntableState;
    }

    public synchronized void setDriveState(TurntableState wantedState) {
        if(turntableState != wantedState) {
            turntableState = wantedState;
        }
    }

    public void setBrakeMode(boolean brake) {
        if(brake != isBrakeMode) {
            isBrakeMode = brake;
            if(brake) {
                turntableMotor.setNeutralMode(NeutralMode.Brake);
            } else {
                turntableMotor.setNeutralMode(NeutralMode.Coast);
            }
        }

    }

    public synchronized void setOpenLoop(double signal) {
        if(turntableState != TurntableState.OPEN_LOOP) {
            turntableState = TurntableState.OPEN_LOOP;
            setBrakeMode(false);
        }
        setMotor(signal);
    }

//encoders
    public double getTurntableEncoder() {//16:140
        //return 0;
        //System.out.println((turntableMotor.getSelectedSensorPosition() / 4098) * (16/140));
        return (double)turntableMotor.getSelectedSensorPosition() / 4098. * (16./140.);
    }

    public TurntableState getTurntableState(){
        return turntableState;
    }

    public void zeroEncoders() {
        turntableMotor.setSelectedSensorPosition(0);
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
            synchronized(Turntable.this){
                switch(turntableState){
                    case OPEN_LOOP:
                        return;
                    case TARGETING:
                        handleTargetting(turntableSetPoint);
                        return;
                    case LIMELIGHT_TARGETING:
                        handleLimelightTargeting();
                        return;
                    default:
                        System.out.println("Unexpected Turntable State" + turntableState);
                }
            }
        }
    };
    public void writeToLog() {
    }
    public void outputToSmartDashboard(){
        SmartDashboard.putNumber("turntable encoder", getTurntableEncoder());
    }
    public void stop(){
        setOpenLoop(0.0);
    }
    public void zeroSensors(){
        zeroEncoders();
    }
    public void registerEnabledLoops(Looper enabledLooper){
        enabledLooper.register(loop);
    }
}