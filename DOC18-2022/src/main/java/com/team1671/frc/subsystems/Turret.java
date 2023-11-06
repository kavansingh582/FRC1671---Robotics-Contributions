package com.team1671.frc.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1671.frc.Constants;
import com.team1671.frc.Ports;
import com.team1671.frc.RobotState;
import com.team1671.frc.Settings;
import com.team1671.frc.subsystems.requests.Request;
import com.team1671.frc.vision.ShooterAimingParameters;
import com.team1671.io.PS5;
import com.team1671.lib.util.Limelight;
import com.team1671.lib.util.PID;
import com.team1671.lib.util.Util;
import com.team254.drivers.LazyTalonFX;
import com.team254.lib.geometry.Rotation2d;
import com.team1671.frc.loops.ILooper;
import com.team1671.frc.loops.Loop;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The turret subsystem controls the direction of the balls being fired. On the turret assembly is the Shooter.
 * The turret can only rotate within 330 degrees but soft limited to 325, the encoder is used only as a backup encoder and to reset
 * the internal encoder of the falcon. This is part of the Superstructure class.
 */
public class Turret extends Subsystem {

    // Motor and Sensors Instantiation
    private LazyTalonFX turret;
    private RobotState robotState;
    private DutyCycle encoder;
    private Limelight limelight = Limelight.getInstance();
    public PID limelightPid = new PID(Constants.Turret.kLimelightP, Constants.Turret.kLimelightI, Constants.Turret.kLimelightD, -1, 1);
    PS5 coDriver;

    PeriodicIO periodicIO = new PeriodicIO();

    // Class Specific Updatable Variables
    public double targetAngle = 0.0;
    private double gyroLockedHeading = 0.0;
    private double gyroLockedTurretHeading = 0.0;
    private double stateEnteredTimestamp = 0.0;
    private double lastTrackWhileShooting = Double.POSITIVE_INFINITY;
    private boolean isEncoderFlipped = false;
    private boolean zeroedAbsolutely = false;

    public double getTargetAngle() {
        return targetAngle;
    }

    private static Turret instance = null;
    public static Turret getInstance() {
        if (instance == null)
            instance = new Turret();
        return instance;
    }

    public State currentState = State.OPEN_LOOP;
    public enum State {
        OPEN_LOOP, VISION, POSITION, GYRO_COMP, AWAITING_LOCK, CRUDE_VISION
    }

    public State getState() {
        return currentState;
    }

    private boolean stateChanged = false;
    private double stateChangeTimestamp = Double.POSITIVE_INFINITY;


    public void setState(State newState) {
        if (newState != currentState) {
            currentState = newState;
            stateChanged = true;
            stateChangeTimestamp = Timer.getFPGATimestamp();
        }
    }

    public Turret() {
        // Setting instantiated motors and sensors
        turret = new LazyTalonFX(Ports.TURRET, "canivore");
        robotState = RobotState.getInstance();
        encoder = new DutyCycle(new DigitalInput(Ports.TURRET_ENCODER));
        // Config TalonFX
        turret.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        turret.enableVoltageCompensation(true);
        turret.setInverted(TalonFXInvertType.Clockwise);        
        turret.configNominalOutputForward(0.0 / 12.0, Constants.kCANTimeoutMs);
        coDriver = new PS5(1);

        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 25, 30, Constants.kCANTimeoutMs);
        turret.configSupplyCurrentLimit(currentLimitConfiguration, Constants.kCANTimeoutMs);

        turret.selectProfileSlot(0, 0);
        turret.config_kP(0, Constants.Turret.kP, Constants.kCANTimeoutMs);
        turret.config_kI(0, Constants.Turret.kI, Constants.kCANTimeoutMs);
        turret.config_kD(0, Constants.Turret.kD, Constants.kCANTimeoutMs);
        turret.config_kF(0, Constants.Turret.kF, Constants.kCANTimeoutMs);

        turret.configMotionCruiseVelocity((int)(Constants.Turret.kMaxSpeed), Constants.kCANTimeoutMs);
        turret.configMotionAcceleration((int)(Constants.Turret.kMaxSpeed * 3.0), Constants.kCANTimeoutMs);
        turret.configMotionSCurveStrength(0);

        turret.configForwardSoftLimitThreshold(turretDegreesToInternalEncUnits(Constants.Turret.kMaxControlAngle), Constants.kCANTimeoutMs);
        turret.configReverseSoftLimitThreshold(turretDegreesToInternalEncUnits(Constants.Turret.kMinControlAngle), Constants.kCANTimeoutMs);
        turret.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
        turret.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);

        System.out.println("Turret max soft limit: " + turretDegreesToInternalEncUnits(Constants.Turret.kMaxControlAngle));
        System.out.println("Turret min soft limit: " + turretDegreesToInternalEncUnits(Constants.Turret.kMinControlAngle));

        turret.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        turret.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 50);

        //setOpenLoop(0.0);
        
        setEncoderPhase(true);
        resetToAbsolute();

    }

    private void setEncoderPhase(boolean phase) {
        isEncoderFlipped = phase;
    }

    private double getAbsoluteEncoderDegrees() {
        return (isEncoderFlipped ? -1.0 : 1.0) * encoder.getOutput() * 360.0;
    }

    public boolean isEncoderConnected() {
        if (RobotBase.isReal()) {
            return (encoder.getFrequency() != 0) ? true : false;
        }
        return false;
    }

    private void setAngle(double angle) {
        targetAngle = boundToTurretRange(angle);
        /*if (ActuatingHood.getInstance().isStowed())
            targetAngle = closestPole();
            */
        periodicIO.controlMode = ControlMode.MotionMagic;
        periodicIO.demand = turretDegreesToInternalEncUnits(targetAngle);
    }

    private boolean inRange(double value, double min, double max) {
        return min <= value && value <= max;
    }

    public boolean inVisionRange(double angle) {
        for (double[] range : Constants.Turret.kVisionRanges) {
            if (inRange(angle, range[0], range[1]))
                return true;
        }
        return false;
    }

    public boolean inVisionRange() {
        return inVisionRange(getAngle());
    }

    public boolean inTurretRange(double angle) {
        angle = Util.boundAngle0to360Degrees(angle);
        if (!inRange(angle, Constants.Turret.kMinControlAngle, Constants.Turret.kMaxControlAngle)) {
            angle = (angle < Constants.Turret.kMinControlAngle) ? (angle + 360.0) : (angle - 360.0);
            return inRange(angle, Constants.Turret.kMinControlAngle, Constants.Turret.kMaxControlAngle);
        }
        return true;
    }

    /** 
     * Takes an angle from any scope and finds the equivalent angle in the turret's range of motion or,
     * if one doesn't exist, returns the turret soft limit closest to the desired angle.
    */
    public double boundToTurretRange(double angle) {
        angle = Util.placeInAppropriate0To360Scope(getAngle(), angle);

        if (!inRange(angle, Constants.Turret.kMinControlAngle, Constants.Turret.kMaxControlAngle)) {
            angle = (angle < Constants.Turret.kMinControlAngle) ? (angle + 360.0) : (angle - 360.0);
            if (inRange(angle, Constants.Turret.kMinControlAngle, Constants.Turret.kMaxControlAngle))
                return angle;
            return (Math.abs(Rotation2d.fromDegrees(angle).distance(Rotation2d.fromDegrees(Constants.Turret.kMinControlAngle)))
                < Math.abs(Rotation2d.fromDegrees(angle).distance(Rotation2d.fromDegrees(Constants.Turret.kMaxControlAngle))))
                ? Constants.Turret.kMinControlAngle : Constants.Turret.kMaxControlAngle;
        }

        return angle;
    }

    public void setPosition(double angle) {
        setState(State.POSITION);
        setAngle(angle);
    }

    public void gyroComp() {
        if (currentState != State.GYRO_COMP) 
            setState(State.GYRO_COMP);
        gyroLockedHeading = robotState.getLatestFieldToVehicle().getValue().getRotation().getDegrees();
        gyroLockedTurretHeading = getAngle();
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getAngle());
    }

    public void lockAngle() {
        setState(State.AWAITING_LOCK);
    }

    private double output = 0.0;
    public void handleCrudeVision(){
        //double output = 0.;
        if(limelight.getTV(0.0) == 1){
            coDriver.rumble(8, 1);
            if(Math.abs(limelight.getTX(0.0)) > 0.05){
                output = limelightPid.calculate(limelight.getTX(0.0), 0.0, Constants.kLooperDt);
                System.out.println("Vision Target Found - TX: " + limelight.getTX(0.0) + "Distance from Target" + limelight.getDistance());
            }else{
                output = 0.0;
                System.out.println("On Target");
            }
        }else{
            output = 0.0;
            System.out.println("No Vision Target Found");
        }
        periodicIO.controlMode = ControlMode.PercentOutput;
        periodicIO.demand = output;
    }

    public void startVision() {
        setState(State.VISION);
        Optional<ShooterAimingParameters> aim = robotState.getAimingParameters();
        //System.out.println(aim.get().getTurretAngle());
        if (aim.isPresent()) {
            System.out.println("Vision Target Range: " + aim.get().getRange() + " Vision Target Angle: " + (double)(aim.get().getTurretAngle().getDegrees()));
            setAngle((double)(aim.get().getTurretAngle().getDegrees()));
        } else {
            System.out.println("No visible or cached target");
        }
    }

    public void setOpenLoop(double output) {
        periodicIO.controlMode = ControlMode.PercentOutput;
        periodicIO.demand = output;
        setState(State.OPEN_LOOP);
    }
    
    public boolean isOpenLoop() {
        return currentState == State.OPEN_LOOP;
    }

    public boolean isTracking() {
        return currentState == State.VISION || currentState == State.CRUDE_VISION;
    }


    
    public boolean hasReachedAngle() {
        return Math.abs(getAngle() - targetAngle) < Constants.Turret.kAngleTolerance;
    }

    public double getAngle() {
        return internalEncUnitsToTurretDegrees(periodicIO.position);
    }

    @Override
    public synchronized void readPeriodicInputs() {
        periodicIO.position = turret.getSelectedSensorPosition(0);
        if (Settings.debugTurret()) {
            periodicIO.velocity = turret.getSelectedSensorVelocity(0);
            periodicIO.voltage = turret.getMotorOutputVoltage();
            periodicIO.current = turret.getOutputCurrent();
        }
    }
    
    @Override
    public void writePeriodicOutputs() {
            turret.set(periodicIO.controlMode, periodicIO.demand);
    }

    private final Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            turret.setNeutralMode(NeutralMode.Brake);
        }

        @Override
        public void onLoop(double timestamp) {
            switch(currentState) {
                case CRUDE_VISION:
                    handleCrudeVision();
                    break;
                case VISION:
                    // Optional<ShooterAimingParameters> aim  = robotState.getAimingParameters();
                    // if (aim.isPresent()) {
                    //     setAngle(aim.get().getTurretAngle().getDegrees());
                    // }
                    break;
                case GYRO_COMP:
                    setAngle(gyroLockedTurretHeading - (robotState.getLatestFieldToVehicle().getValue().getRotation().getDegrees() - gyroLockedHeading));
                    break;
                case AWAITING_LOCK:
                    if (stateChanged) {
                        periodicIO.controlMode = ControlMode.PercentOutput;
                        periodicIO.demand = 0.0;
                    }
                    if (timestamp - stateChangeTimestamp > 0.25)
                        setPosition(getAngle());
                default:
                break;
            }
        }

        @Override
        public void onStop(double timestamp) {
            turret.setNeutralMode(NeutralMode.Coast);
        }
    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Turret Angle", getAngle());
        SmartDashboard.putNumber("Turret Absolute Angle", getAbsoluteEncoderDegrees());
        SmartDashboard.putNumber("Turret Velocity", periodicIO.velocity);
        //SmartDashboard.putNumber("Limelight guessed angle", robotState.getAimingParameters().get().getTurretAngle().getDegrees());
        //SmartDashboard.putNumber("turret current", turret.getOutputCurrent());
        //SmartDashboard.putNumber("Turret Velocity", periodicIO.velocity);
        if (Settings.debugTurret()) {
            SmartDashboard.putString("Turret State", currentState.toString());
            SmartDashboard.putNumber("Turret Absolute Frequency", encoder.getFrequency());
            SmartDashboard.putNumber("Turret Encoder", periodicIO.position);
            SmartDashboard.putNumber("Turret Velocity", periodicIO.velocity);
            SmartDashboard.putNumber("Turret Angle Error", internalEncUnitsToTurretDegrees(turret.getClosedLoopError(0)));
            if (turret.getControlMode() == ControlMode.MotionMagic) {
                SmartDashboard.putNumber("Turret Setpoint", turret.getClosedLoopTarget(0));
            }
        }
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    public Request angleRequest(double angle, double speedScalar, boolean waitForAngle) {
        return new Request(){
            
            @Override
            public void act() {
                turret.configMotionCruiseVelocity((int)(Constants.Turret.kMaxSpeed * speedScalar), Constants.kCANTimeoutMs);
                setPosition(angle);
            }
            
            @Override
            public boolean isFinished() {
                if (waitForAngle) {
                    return hasReachedAngle();
                } else {
                    return true;
                }
            }
            
        };
    }
    
    public Request angleRequest(double angle) {
        return angleRequest(angle, 1.0, true);
    }
    
    public Request angleRequest(double angle, double speedScalar) {
        return angleRequest(angle, speedScalar, true);
    }

    public Request safeLockAngleRequest() {
        return new Request(){
        
            @Override
            public void act() {
                setOpenLoop(0.0);
            }
        };
    }
    
    public Request lockAngleRequest() {
        return new Request(){
            
            @Override
            public void act() {
                lockAngle();
            }
        };
    }

    public Request startVisionRequest() {
        return new Request(){
            
            @Override
            public void act() {
                setState(State.CRUDE_VISION);           
            }
            
      
      
        };
    }
public Request openLoopRequest(double signal){
    return new Request(){
            
        @Override
        public void act() {
            setOpenLoop(signal);
        }
    };
}


    public Request stateRequest(State desiredState) {
        return new Request(){
        
            @Override
            public void act() {
                setState(desiredState);
            }
        };
    }

    public Request crudeVisionRequest(){
        return new Request(){
            @Override
            public void act(){
                handleCrudeVision();
            }
        };
    }

    public int turretDegreesToInternalEncUnits(double turretAngle) {
        return (int) ((turretAngle / 360.0) * Constants.Turret.kInternalEncToOutputRatio * 2048.0);
    }

    public double internalEncUnitsToTurretDegrees(double encUnits) {
        return (encUnits / 2048.0) / Constants.Turret.kInternalEncToOutputRatio * 360.0;
    }

    public synchronized void resetToAbsolute() {
        if (!zeroedAbsolutely) {
            //System.out.println("Turret Encoder Connected: " + isEncoderConnected());
            if (isEncoderConnected() && RobotBase.isReal()) {
                //DriverStation.reportError("TURRET WAS RESET TO ABSOLUTE WITH THE MAG ENCODER", false);
                double absolutePosition = Util.boundAngle0to360Degrees(getAbsoluteEncoderDegrees() - Constants.Turret.kEncoderStartingAngle);
                if (absolutePosition > Constants.Turret.kMaxInitialAngle)
                    absolutePosition -= 360.0;
                else if (absolutePosition < Constants.Turret.kMinInitialAngle)
                    absolutePosition += 360.0;
                if (!inRange(absolutePosition, Constants.Turret.kMinInitialAngle, Constants.Turret.kMaxInitialAngle)) {
                    DriverStation.reportError("Turret angle is out of bounds", false);
                    hasEmergency = true;
                }
                turret.setSelectedSensorPosition(turretDegreesToInternalEncUnits(absolutePosition), 0, Constants.kCANTimeoutMs);
                //System.out.println("Turret Absolute angle: " + getAbsoluteEncoderDegrees() + ", encoder offset: " + Constants.Turret.kEncoderStartingAngle + ", difference: " + (getAbsoluteEncoderDegrees() - Constants.Turret.kEncoderStartingAngle) + ", degreesToEncUnits: " + turretDegreesToInternalEncUnits(getAbsoluteEncoderDegrees() - Constants.Turret.kEncoderStartingAngle));
            } else {
                DriverStation.reportError("Turret encoder NOT DETECTED: CURRENT POSITION SET TO 0", false);
                turret.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
            }
        }
    }

    public synchronized boolean getCanFold() {
        return inRange(getAngle(), -15.0, 15.0) || inRange(getAngle(), 165.0, 195.0);
    }

    public double closestPole() {
        return (Math.abs(getAngle() - 180.0) < Math.abs(getAngle() - 0.0)) ? 180.0 : 0.0;
    }

    public void setNearestFoldAngle() {
        setPosition(closestPole());
    }

    public boolean isGoingToPole() {
        return Util.epsilonEquals(targetAngle, 0.0) || Util.epsilonEquals(targetAngle, 180.0);
    }

    public synchronized void zeroedTurret() {
        System.out.println("Turret Zeroed");
        System.out.println("Turret Absolute angle: " + getAbsoluteEncoderDegrees() + ", encoder offset: " + Constants.Turret.kEncoderStartingAngle + ", difference: " + (getAbsoluteEncoderDegrees() - Constants.Turret.kEncoderStartingAngle) + ", degreesToEncUnits: " + turretDegreesToInternalEncUnits(getAbsoluteEncoderDegrees() - Constants.Turret.kEncoderStartingAngle));
        zeroedAbsolutely = true;
    }

    private static class PeriodicIO {
        //Inputs
        public double position;
        public double velocity;
        public double voltage;
        public double current;
        
        //Outputs
        public double demand = 0.0;
        public ControlMode controlMode = ControlMode.PercentOutput;
    }

}
