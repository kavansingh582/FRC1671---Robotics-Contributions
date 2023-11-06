package frc.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import drivers.LazyTalonFX;
import frc.loops.Loop;
import frc.loops.Looper;
import frc.robot.Constants;
import frc.util.DriveSignal;
import frc.util.Util;
import trajectory_lib.*;


public class Drive extends Subsystem{
    private static Drive instance;
    public static Drive getInstance() {
        if(instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    public enum DriveState {
        OPEN_LOOP,
        PATH_FOLLOWING,
        TURN_IN_PLACE
    }

    private TalonFX leftFront, leftRear, rightFront, rightRear;
    private TrajectoryFollower trajectoryFollower;
    private TrajectoryConfig config;
    private boolean reversed = false;
    private double absAngleSetpoint;
    private HeadingController headingController;
    private PigeonIMU imu;
    private double angleOffset = 0.;


    private boolean isBrakeMode;

    private DriveState driveState = DriveState.OPEN_LOOP;

    private Drive() {
        //make
        leftFront =  new LazyTalonFX(Constants.LEFT_FRONT);
        leftRear =  new LazyTalonFX(Constants.LEFT_REAR);
        rightFront =  new LazyTalonFX(Constants.RIGHT_FRONT);
        rightRear =  new LazyTalonFX(Constants.RIGHT_REAR);

        //follower motors
        leftRear.set(TalonFXControlMode.Follower, Constants.LEFT_FRONT);
        rightRear.set(TalonFXControlMode.Follower, Constants.RIGHT_FRONT);

        //encoders
        leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.LONG_CAN_TIMEOUT_MS);
        rightFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.LONG_CAN_TIMEOUT_MS);
        leftFront.setSelectedSensorPosition(0);
        rightFront.setSelectedSensorPosition(0);
        imu = new PigeonIMU(Constants.DRIVE_IMU);

        //voltage
        leftFront.enableVoltageCompensation(true);
        rightFront.enableVoltageCompensation(true);

        leftFront.configVoltageCompSaturation(10.0, Constants.LONG_CAN_TIMEOUT_MS);
        rightFront.configVoltageCompSaturation(10.0, Constants.LONG_CAN_TIMEOUT_MS);
        
        leftFront.configOpenloopRamp(0.0, Constants.LONG_CAN_TIMEOUT_MS);
        rightFront.configOpenloopRamp(0.0, Constants.LONG_CAN_TIMEOUT_MS);

        //path following
        trajectoryFollower = new TrajectoryFollower(Constants.TRAJECTORY_FOLLOWER_KP, Constants.TRAJECTORY_FOLLOWER_KI, Constants.HEADING_CONTROLLER_KD, Constants.TRAJECTORY_FOLLOWER_KV,Constants.TRAJECTORY_FOLLOWER_KA, Constants.TRAJECTORY_FOLLOWER_KW);
        config = new TrajectoryConfig(TrajectoryConfig.SAMPLES_FAST, 0.005, Constants.DRIVE_ALLOWED_SPEED, Constants.DRIVE_MAX_ACC, Constants.DRIVE_MAX_JERK);
        trajectoryFollower.getHeadingController().setInputRange(-Math.PI, Math.PI);
        trajectoryFollower.getHeadingController().setPID(Constants.HEADING_CONTROLLER_KP, Constants.HEADING_CONTROLLER_KI, Constants.HEADING_CONTROLLER_KD);
        headingController = new HeadingController(Constants.HEADING_CONTROLLER_KP, Constants.HEADING_CONTROLLER_KI, Constants.HEADING_CONTROLLER_KD);

        //other
        isBrakeMode = true;
        absAngleSetpoint = 0.0;
    }

    //path following
    public TrajectoryConfig getConfig(){
        return config;
    }
    
    public void setTrajectory(AutoTrajectory traj, boolean reversed) {
        trajectoryFollower.setTrajectory(traj);
        this.reversed = reversed;
    }
    
    public synchronized void startPathFollowing() {
        if(driveState != DriveState.PATH_FOLLOWING) {
            driveState = DriveState.PATH_FOLLOWING;
            trajectoryFollower.start(getLeftEncoder(), getRightEncoder(), reversed);
        }
    }
    
    private void handlePathFollowing() {//NEGATIVE HERE
        DriveSignal signal = trajectoryFollower.calculate(getLeftEncoder(), getLeftEncoderRate(), getRightEncoder(), getRightEncoderRate(), getHeading(), Constants.LOOPER_DT);
        DriveSignal signalToMotor = new DriveSignal(signal.getLeft(), signal.getRight());
            
        setMotors(signalToMotor);
        System.out.println("left: " + signalToMotor.getLeft() + "  right: " + signalToMotor.getRight());
    }

    public boolean pathIsFinished() {
        return trajectoryFollower.isFinished();
    }

    public synchronized void setAbsoluteAngle(double angleToSet) {
        absAngleSetpoint = angleToSet;
        headingController.reset();
    }

    public synchronized void setRelativeAngle(double relativeAngle) {
        absAngleSetpoint = getHeading() + relativeAngle;
        headingController.reset();
    }

    public synchronized double getAbsAngleSetpoint() {
        return absAngleSetpoint;
    }

    private void handleTurnInPlace() { //NEGATIVE HERE
        double powerToTurn = headingController.calculate(Utils.boundHalfDegrees(absAngleSetpoint), Utils.boundHalfDegrees(getHeading()), Constants.LOOPER_DT);
        DriveSignal signal = new DriveSignal(powerToTurn, -powerToTurn); 
        if(Util.epsilonEquals(getHeading(), absAngleSetpoint, 1.0)) {
            signal = DriveSignal.NEUTRAL;
        }else{
        }
        
        setMotors(signal);
    }
    
    //state
    public synchronized void setDriveState(DriveState wantedState) {
        if(driveState != wantedState) {
            driveState = wantedState;
        }
    }
    
    public DriveState getDriveState() {
        return driveState;
    }
    
    //motors
    public synchronized void setRampRate(double rampRate) {
        leftFront.configOpenloopRamp(rampRate, Constants.LONG_CAN_TIMEOUT_MS);
        rightFront.configOpenloopRamp(rampRate, Constants.LONG_CAN_TIMEOUT_MS);
    }

    public void setMotors(DriveSignal signal) { //WHAT SHOULD ACTUALLY CHANGE
        leftFront.set(TalonFXControlMode.PercentOutput, -signal.getLeft());
        rightFront.set(TalonFXControlMode.PercentOutput, signal.getRight());
    }
    
    public synchronized void setBrakeMode(boolean on) {
        if(isBrakeMode != on) {
            isBrakeMode = on;
            if(on) {
                leftFront.setNeutralMode(NeutralMode.Brake);
                leftRear.setNeutralMode(NeutralMode.Brake);

                rightFront.setNeutralMode(NeutralMode.Brake);
                rightRear.setNeutralMode(NeutralMode.Brake);
            } else {
                leftFront.setNeutralMode(NeutralMode.Coast);
                leftRear.setNeutralMode(NeutralMode.Coast);

                rightFront.setNeutralMode(NeutralMode.Coast);
                rightRear.setNeutralMode(NeutralMode.Coast);
            }
        }
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if(driveState != DriveState.OPEN_LOOP) {
            driveState = DriveState.OPEN_LOOP;
            setBrakeMode(false);
        }
        setMotors(signal);
    }

    public double getLeftEncoder() { //ft gear ratio is (60/12) * (28/20) or just 7/1
        //return leftFront.getSelectedSensorPosition() * (((Constants.WHEEL_DIAMETER * Math.PI) / 1440) / 12);
        return -1 * leftFront.getSelectedSensorPosition() / 7 * ((Constants.WHEEL_DIAMETER * Math.PI) / Constants.FALCON_PULSE_PER_REV) / 12;
    }
    public double getRightEncoder() {
        //return rightFront.getSelectedSensorPosition() * (((Constants.WHEEL_DIAMETER * Math.PI) / 1440) / 12);
        return rightFront.getSelectedSensorPosition() / 7 * ((Constants.WHEEL_DIAMETER * Math.PI) / Constants.FALCON_PULSE_PER_REV) / 12;
    }
    public double getLeftEncoderRate() {
        return -1*((leftFront.getSelectedSensorVelocity() / 7 * (((Constants.WHEEL_DIAMETER * Math.PI) / Constants.FALCON_PULSE_PER_REV) / 12)))*10;
    }
    public double getRightEncoderRate() {
        return ((rightFront.getSelectedSensorVelocity() / 7 * (((Constants.WHEEL_DIAMETER * Math.PI) / Constants.FALCON_PULSE_PER_REV) / 12)))*10;
    }

    //gyro
    public void zeroGyro() {
        PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
        angleOffset = imu.getFusedHeading(fusionStatus);
    }

    public double getHeading() {
        PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
        return imu.getFusedHeading(fusionStatus) - angleOffset;
    }

    public double getHeadingRate() {
        double[] rpy = new double[3];
		imu.getRawGyro(rpy);
        return rpy[2];
    }

    public void zeroEncoders(){
        leftFront.setSelectedSensorPosition(0);
        rightFront.setSelectedSensorPosition(0);
        zeroGyro();
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
            synchronized(Drive.this){
                switch(driveState){
                    case OPEN_LOOP:
                        return;
                    case PATH_FOLLOWING:
                        handlePathFollowing();
                        return;
                    case TURN_IN_PLACE:
                        handleTurnInPlace();
                        return;
                    default:
                        System.out.println("Unexpected Drive Control State: " + driveState);
                }
            }
        }
    };
    public void writeToLog() {
    }
    public void outputToSmartDashboard(){
        // SmartDashboard.putNumber("Right Encoder", getRightEncoder());
        // SmartDashboard.putNumber("Left Encoder", getLeftEncoder());
        // SmartDashboard.putNumber("Right Encoder Rate", getRightEncoderRate());
        // SmartDashboard.putNumber("Left Encoder Rate", getLeftEncoderRate());

        // SmartDashboard.putNumber("Right front Drive Voltage", rightFront.getMotorOutputVoltage());
        // SmartDashboard.putNumber("Left front Drive Voltage", leftFront.getMotorOutputVoltage());
        // SmartDashboard.putNumber("Right rear Drive Voltage", rightRear.getMotorOutputVoltage());
        // SmartDashboard.putNumber("Left rear Drive Voltage", leftRear.getMotorOutputVoltage());
    }
    public void stop(){
        setOpenLoop(DriveSignal.NEUTRAL);
    }
    public void zeroSensors(){
        zeroEncoders();
    }
    public void registerEnabledLoops(Looper enabledLooper){
        enabledLooper.register(loop);
    }
}