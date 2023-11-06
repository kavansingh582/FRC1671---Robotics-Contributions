package frc.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.Flywheel;
import frc.subsystems.Intake;

public class ShootAction implements Action {
    private Flywheel flywheel = Flywheel.getInstance();
    private Intake intake = Intake.getInstance();
    private double shootVel;
    private double revupTime;
    private double timeout = 1.0;
    private double startTime;


    public ShootAction(double shootVel, double timeout, double revupTime) {
        this.shootVel = shootVel;
        this.timeout = timeout;
        this.revupTime = revupTime;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();

        Flywheel.FlywheelState wantedState = flywheel.new FlywheelState();
        wantedState.vel = shootVel;

        flywheel.setWantedState(wantedState);
    }

    @Override
    public void update() {
        Intake.IntakeState wantedIntakeState = intake.new IntakeState();
        double currentTime = Math.abs(Timer.getFPGATimestamp() - startTime);
        if(currentTime <= timeout + revupTime - timeout){
            wantedIntakeState.beltVel = 0.0;
        }else{
            wantedIntakeState.beltVel = 0.5;
        }
        intake.setWantedState(wantedIntakeState);
    }

    @Override
    public boolean isFinished() { 
        return Timer.getFPGATimestamp() - startTime >= timeout;        
    }

    @Override
    public void done() {
        Flywheel.FlywheelState wantedState = flywheel.new FlywheelState();
        wantedState.vel = 0.0;

        Intake.IntakeState wantedIntakeState = intake.new IntakeState();
        wantedIntakeState.beltVel = 0.;
        wantedIntakeState.intakeBarVel = 0.;


        flywheel.setWantedState(wantedState);
    }
    
}