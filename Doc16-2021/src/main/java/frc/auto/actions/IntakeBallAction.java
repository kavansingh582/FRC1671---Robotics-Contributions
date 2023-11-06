package frc.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.Intake;

public class IntakeBallAction implements Action {
    private Intake intake = Intake.getInstance();
    private double intakeVel;
    private double timeout = 1.0;
    private double startTime;


    public IntakeBallAction(double intakeVel, double timeout) {
        this.intakeVel = intakeVel;
        this.timeout = timeout;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();

        Intake.IntakeState wantedState = intake.new IntakeState();
        wantedState.intakeBarVel = intakeVel;
        wantedState.beltVel = intakeVel;
        wantedState.hopperVel = intakeVel;
        wantedState.isExtended = true;

        intake.setWantedState(wantedState);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        
        return Timer.getFPGATimestamp() - startTime >= timeout;
        
        
    }

    @Override
    public void done() {
        Intake.IntakeState wantedState = intake.new IntakeState();
        wantedState.intakeBarVel = 0.0;

        intake.setWantedState(wantedState);
    }
    
}