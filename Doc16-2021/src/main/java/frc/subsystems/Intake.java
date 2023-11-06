package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import drivers.LazyTalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.loops.*;
import frc.robot.Constants;

public class Intake extends Subsystem {
    private static Intake instance;
    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public class IntakeState{
        public double intakeBarVel = 0.;
        public double hopperVel = 0.;
        public double beltVel = 0.;
        public boolean isExtended = false;
    }
    
    public TalonSRX beltMotor, intakeBarMotor, hopperMotor;
    public DoubleSolenoid intakeSol;

    IntakeState wantedState = new IntakeState();
    IntakeState state = new IntakeState();

    private Intake(){
        intakeBarMotor = new LazyTalonSRX(Constants.INTAKE_BAR_MOTOR);
        hopperMotor = new LazyTalonSRX(Constants.HOPPER_MOTOR);        
        beltMotor = new LazyTalonSRX(Constants.BELT_MOTOR);
        
        intakeBarMotor.enableVoltageCompensation(true);
        intakeBarMotor.configVoltageCompSaturation(12, Constants.LONG_CAN_TIMEOUT_MS);
        beltMotor.enableVoltageCompensation(true);
        beltMotor.configVoltageCompSaturation(12, Constants.LONG_CAN_TIMEOUT_MS);
        hopperMotor.enableVoltageCompensation(true);
        hopperMotor.configVoltageCompSaturation(12, Constants.LONG_CAN_TIMEOUT_MS);
        
        intakeSol = new DoubleSolenoid(Constants.INTAKE_SOL_1, Constants.INTAKE_SOL_2);        
    }

    public void setMotors(double bar, double hopper, double belt){
        beltMotor.set(ControlMode.PercentOutput, -belt);
        intakeBarMotor.set(ControlMode.PercentOutput, -bar);
        hopperMotor.set(ControlMode.PercentOutput, hopper);
    }

    

    public synchronized void setWantedState(IntakeState wanted) {
        wantedState = wanted;
    }

    public synchronized void IntakeHandler(IntakeState wanted, IntakeState current, double timestamp) {
        setMotors(wanted.intakeBarVel, wanted.hopperVel, wanted.beltVel);
        extendIntake(wanted.isExtended);
    }

    public void extendIntake(boolean extend) {
        if(extend != state.isExtended) {
            state.isExtended = extend;
            if(extend) {
                intakeSol.set(Value.kReverse); 
            } else {
                intakeSol.set(Value.kForward);
            }
        }   
    }

    public IntakeState getState(){
        return state;
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
            synchronized(Intake.this){
                IntakeHandler(wantedState, state, timestamp);
            }
        }
    };

    public void writeToLog() {
    }
    public void outputToSmartDashboard(){
    }
    public void stop(){
        setMotors(0., 0., 0.);
        extendIntake(false);
    }
    public void zeroSensors(){
    }
    public void registerEnabledLoops(Looper enabledLooper){
        enabledLooper.register(loop);
    }
}