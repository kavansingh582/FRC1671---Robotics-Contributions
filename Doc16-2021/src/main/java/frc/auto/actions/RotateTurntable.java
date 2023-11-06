package frc.auto.actions;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.Turntable;

public class RotateTurntable implements Action{
    private Turntable turntable = Turntable.getInstance();
    private double setPoint = 0.0;
    private boolean goalMet;
    private double startTime, timeout = 0.0;
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    double tx = limelight.getEntry("tx").getDouble(0.);

    public RotateTurntable(double setPoint, double timeout){
        this.setPoint = setPoint;
        this.timeout = timeout;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
        turntable.startTargetting(setPoint);
    }
    
    @Override
    public void update() {
    }

    @Override
    public void done() {
        //turntable.setOpenLoop(signal);
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(turntable.getTurntableEncoder() - setPoint) < 1.0){
            goalMet = true;
        } else{
            goalMet = false;
        }
        return Timer.getFPGATimestamp() - startTime >= timeout || goalMet;
    }
    
}