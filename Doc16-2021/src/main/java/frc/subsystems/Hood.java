package frc.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.loops.Loop;
import frc.loops.Looper;
import frc.robot.Constants;

public class Hood extends Subsystem{
    private static Hood instance;

    public static Hood getInstance() {
        if(instance == null) {
            instance = new Hood();
        }
        return instance;
    }
    private DoubleSolenoid hood;
    private Compressor compressor;
    HoodState wantedState = new HoodState();
    HoodState state = new HoodState();

    private Hood() {
        hood = new DoubleSolenoid(Constants.COMPRESSOR ,Constants.HOOD_SOL_2, Constants.HOOD_SOL_1);
        compressor = new Compressor(Constants.COMPRESSOR);
    }

    public class HoodState {
        public boolean isExtended = false;
    }

    public void handleHood(HoodState wanted, HoodState state) {
        extendHood(wanted.isExtended);
    }

    public HoodState getState(){
        return state;
    }

    public void extendHood(boolean extend) {
        if(extend != state.isExtended) {
            state.isExtended = extend;
            if(extend) {
                hood.set(Value.kForward);
            }else{
                hood.set(Value.kReverse);
            }
        }
    }

    Loop loop = new Loop(){
    
        @Override
        public void onStop(double timestamp) {
            stop();
        }
    
        @Override
        public void onStart(double timestamp) {
            extendHood(false);
            compressor.start();
        }
    
        @Override
        public void onLoop(double timestamp) {
            synchronized(Hood.this) {
                handleHood(wantedState, state);
                if(compressor.getPressureSwitchValue() == false){
                    compressor.start();
                }else{
                    compressor.stop();
                }
            }
        }
    };
    public synchronized void setWantedState(HoodState wanted) {
        wantedState = wanted;
    }

    public void writeToLog() {
    };
    public void outputToSmartDashboard() {
    }
    public void stop() {
        hood.set(DoubleSolenoid.Value.kReverse);
    }
    public void zeroSensors() {
    }
    public void registerEnabledLoops(Looper enabledLooper){
        enabledLooper.register(loop);
    }
}