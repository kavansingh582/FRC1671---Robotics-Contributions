package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import drivers.LazyTalonSRX;
import frc.loops.Loop;
import frc.loops.Looper;
import frc.robot.Constants;

public class ControlPanel extends Subsystem {
    private static ControlPanel instance;
    public static ControlPanel getInstance() {
        if (instance == null) {
            instance = new ControlPanel();
        }
        return instance;
    }

    public enum ControlPanelState {
        OPEN_LOOP,
        AUTO
    }

    private TalonSRX controlPanelMotor, controlPanelFollower;
    private ControlPanelState state = ControlPanelState.OPEN_LOOP;
    private ControlPanelState wanted = ControlPanelState.OPEN_LOOP;


    private ControlPanel(){
        controlPanelMotor = new LazyTalonSRX(Constants.CONTROL_PANNEL_MOTOR);
        controlPanelFollower = new LazyTalonSRX(Constants.CONTROL_PANNEL_FOLLOWER);
        controlPanelFollower.set(ControlMode.Follower, Constants.CONTROL_PANNEL_MOTOR);
    }

    public void setMotor(double signal){
        controlPanelMotor.set(ControlMode.PercentOutput, signal);
    }

    public synchronized void handleAutomaticControlPanel(ControlPanelState wanted, ControlPanelState current){
        
    }

    public synchronized void setOpenLoop(double signal){
        if(state != ControlPanelState.OPEN_LOOP){
            state = ControlPanelState.OPEN_LOOP;
        }
        setMotor(signal);
    }
    Loop loop = new Loop(){
    
        @Override
        public void onStop(double timestamp) {
            stop();
        }
    
        @Override
        public void onStart(double timestamp) {
            
        }
    
        @Override
        public void onLoop(double timestamp) {
            synchronized(ControlPanel.this){
                switch(state){
                    case OPEN_LOOP:
                        return;
                    case AUTO:
                        handleAutomaticControlPanel(wanted, state);
                    default:
                    System.out.println("Unexpected Turntable State" + state);
                }
            }
        }
    };

    public ControlPanelState getTurntableState() {
        return state;
    }

    public void outputToSmartDashboard() {
    }
    public void stop() {
        setMotor(0.0);
    }
    public void zeroSensors() {
    }
	public void registerEnabledLoops(Looper enabledLooper) {	
        enabledLooper.register(loop);	
    }
}
