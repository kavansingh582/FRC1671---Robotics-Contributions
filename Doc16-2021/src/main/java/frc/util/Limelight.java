package frc.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

public class Limelight {

    private static Limelight instance;
    public static Limelight getInstance() {
        if(instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    public static final double HEIGHT_OF_TARGET = 98.25;//in
    public static final double HEIGHT_OF_LIMELIGHT = 38.0;//in
    public static final double LIMELIGHT_MOUNTING_ANGLE = 30.0;//deg

    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public double getTV(double defaultValue){
        return limelight.getEntry("tv").getDouble(defaultValue);
    }

    //coordinates
    public double getTX(double defaultValue){
        return limelight.getEntry("tx").getDouble(defaultValue);
    }

    public double getTY(double defaultValue){
        return limelight.getEntry("ty").getDouble(defaultValue);
    }

    public double getTS(double defaultValue){
        return limelight.getEntry("ts").getDouble(defaultValue);
    }

    //geometry
    public double getTargetArea(double defaultValue){
        return limelight.getEntry("ta").getDouble(defaultValue);
    }

    public double getShortestSide(double defaultValue){
        return limelight.getEntry("tshort").getDouble(defaultValue);
    }

    public double getLongestSide(double defaultValue){
        return limelight.getEntry("tlong").getDouble(defaultValue);
    }

    public double getHorizontalSidelength(double defaultValue){
        return limelight.getEntry("thor").getDouble(defaultValue);
    }

    public double getVerticalSideLength(double defaultValue){
        return limelight.getEntry("tvert").getDouble(defaultValue);
    }

    public double getCamTran(double defaultValue){
        return limelight.getEntry("camtran").getDouble(defaultValue);
    }

    //sets

    /**
     * Set LED modes for limelight
     * @param mode 0 is the one in the pipeline, 1 is frce off, 2 is force blink, 3 is force on
     */
    public void setLedMode(int mode){
        limelight.getEntry("ledMode").setNumber(mode);
    }

    /**
     * Set cam mode
     * @param mode 0 is normal vision procesing, 1 is driver camera
     */
    public void setCamMode(int mode){
        limelight.getEntry("camMode").setNumber(mode);
    }

    //usb
    public double getDistance(){
        //a2 is the y angle to the target (ty?)
        //double limelightAngleToTarget = getTY(0.0);
        //this equation can be found in the limelight documentation in the "estimating distance" section
        return (HEIGHT_OF_TARGET - HEIGHT_OF_LIMELIGHT) / Math.tan(Math.toRadians(LIMELIGHT_MOUNTING_ANGLE + getTY(0.0)));
    }
}