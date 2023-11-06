package com.team1671.frc;

import java.util.Arrays;
import java.util.List;

import com.team1671.lib.util.InterpolatingTreeMap;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team1671.lib.util.InterpolatingDouble;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class Constants {
    /*All distance measurements are in inches, unless otherwise noted.*/
    
    public static final double kLooperDt = 0.02;
    
    public static final double kEpsilon = 0.0001;
    
    //Physical Robot Dimensions (including bumpers)
    public static final double kRobotWidth = 34.75;
    public static final double kRobotLength = 34.75;
    public static final double kRobotHalfWidth = kRobotWidth / 2.0;
    public static final double kRobotHalfLength = kRobotLength / 2.0;
    
    
    
    //start poses
    //red top
    public static final Pose2d quartileOneStartingPose = new Pose2d(new Translation2d(357.97 - 10.691 ,   -25.317 - 23.879), Rotation2d.fromDegrees(-69.0));//Q1
    //red bottom
    public static final Pose2d quartileTwoStartingPose = new Pose2d(new Translation2d(375.31, 22.61), Rotation2d.fromDegrees(21.0));//Q2
    //blue bottom
    public static final Pose2d quartileThreeStartingPose = new Pose2d(new Translation2d(302.11, 52.27), Rotation2d.fromDegrees(-69.+180.));
    //blue top
    public static final Pose2d quartileFourStartingPose = new Pose2d(new Translation2d(271.48, -21.64), Rotation2d.fromDegrees(21.0+180.0));
    //test
    public static final Pose2d testStartPose = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0.0));


    
    /*
    * Target Specifications
    */
    public static final double kVisionTargetHeight = 103.62; //81.0 to bottom
    
    //Swerve Calculations Constants (measurements are in inches)
    public static final double kWheelbaseLength = 34.75;
    public static final double kWheelbaseWidth = 34.75;
    public static final double kSwerveDiagonal = Math.hypot(kWheelbaseLength, kWheelbaseWidth);
    
    //Camera Constants (X and Y are with respect to the turret's center)
    public static final double kCameraYOffset = 0.0;//0.25
    public static final double kCameraXOffset = 9.313305; //8.5
    public static final double kCameraZOffset = 8.685829; //26.776 24.524
    public static final double kCameraYawAngleDegrees = 0;//-12.7
    public static final double kCameraPitchAngleDegrees = 35; //degrees

    
    //Limelight
    public static final double kLimelightHeight = 41.25; // degrees
    public static final double kLimelightMountingAngle = 35; // degrees
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
    
    //Goal tracker constants
    public static final double kMaxGoalTrackAge = 0.5;
    public static final double kMaxTrackerDistance = 18.0;
    public static final double kCameraFrameRate = 90.0;
    public static final double kTrackStabilityWeight = 1.0;
    public static final double kTrackAgeWeight = 1.0;
    public static final double kTrackSwitchingWeight = 0.0;
    public static final double kClosestVisionDistance = 26.0;//36.0
    
    public static final double kVisionPIDOutputPercent = 0.5;

    public static final double kPosePredictionTime = 0.125; // seconds 0.25
    
    public static final double kDistanceToTargetTolerance = 1.0;

    public static final double kGyroDriftPerRotation = -0.25; // degrees
    
    //Path following constants
    public static final double kPathLookaheadTime = 0.25;  // seconds to look ahead along the path for steering 0.4
    public static final double kPathMinLookaheadDistance = 6.0;  // inches 24.0 (we've been using 3.0)
    
    //Swerve Speed Constants
    public static final double kSwerveDriveMaxSpeed = 22570.0; //23000 //ticks/s (measure in phoenix tuner)
    public static final double kSwerveMaxSpeedInchesPerSecond = 12.5 * 12.0;
    public static final double kSwerveRotationMaxSpeed = 22430.0 * 0.8; //The 0.8 is to request a speed that is always achievable | ticks/s (measure in phoenix tuner)
    public static final double kSwerveRotation10VoltMaxSpeed = 1350.0;
    public static final double kSwerveRotationSpeedScalar = ((1.0 / 0.125) - 1.0) / kSwerveMaxSpeedInchesPerSecond;
    
    //Swerve Module Wheel Offsets (Rotation encoder values when the wheels are facing 0 degrees)
    /**
    * To Zero: Rotate module so that bevel gear is face out. Rotate module 90° CW from the top
    * Enter angle read by the absolute encoder. Insert as degrees and subtract or add 90° to the value
    * based on where the bevel ended up.
    * these are your module absolute angle
    */
    public static final double kFrontRightEncoderStartingPos = -289.0083;
    public static final double kFrontLeftEncoderStartingPos = -93.6252;
    public static final double kRearLeftEncoderStartingPos = -42.6015;
    public static final double kRearRightEncoderStartingPos = -122.0602; 
    
    //Swerve Module Positions (relative to the center of the drive base)][\
    
    public static final Translation2d kVehicleToModuleZero = new Translation2d(kWheelbaseLength / 2, kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleOne = new Translation2d(kWheelbaseLength / 2, -kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleTwo = new Translation2d(-kWheelbaseLength / 2, -kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleThree = new Translation2d(-kWheelbaseLength / 2, kWheelbaseWidth / 2);
    
    public static final List<Translation2d> kModulePositions = Arrays.asList(kVehicleToModuleZero,
    kVehicleToModuleOne, kVehicleToModuleTwo, kVehicleToModuleThree);
    
    //Scrub Factors
    public static final boolean kSimulateReversedCarpet = false;
    public static final double[] kWheelScrubFactors = new double[]{1.0, 1.0, 1.0, 1.0};
    public static final double kXScrubFactor = 1.0;//1.0 / (1.0 - (9549.0 / 293093.0));
    public static final double kYScrubFactor = 1.0;//1.0 / (1.0 - (4.4736 / 119.9336));
    
    //Voltage-Velocity equation constants {m, b, x-intercept}
    //First set is the positive direction, second set is negative
    public static final double[][][] kVoltageVelocityEquations = new double[][][]{
        {{1.70, -4.39, 2.58}, {1.83, 5.23, -2.85}},
        {{1.59, -3.86, 2.42}, {1.43, 3.09, -2.16}},
        {{1.53, -3.66, 2.39}, {1.66, 4.15, -2.50}},
        {{1.84, -4.70, 2.56}, {1.85, 5.34, -2.89}}
    };
    
    //Swerve Odometry Constants
    public static final double kSwerveWheelDiameter = 4.09; //inches (actual diamter is closer to 3.87, but secondary algorithm prefers 4.0901) 3.76
    public static final double kSwerveDriveEncoderResolution = 2048.0; //2048.0 for falcon 500
    public static final double kSwerveRotationEncoderResolution = 2048.0;
    /** The number of rotations the swerve rotat
     *
     * ion motor undergoes for every rotation of the module. */
    public static final double kSwerveRotationReduction = 15.4286; // 8/24 18/72
    /** The number of rotations the swerve drive encoder undergoes for every rotation of the wheel. */
    public static final double kSwerveEncoderToWheelRatio = 54/6; // 10/40 28/20 15/45 IN REVERSE //16.8
    public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * kSwerveEncoderToWheelRatio;
    public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);

    //swerve rotation PID
    //normal
    public static final double kSwerveRotationP0 = 1.55; //1.55
    public static final double kSwerveRotationI0 = 0.0;
    public static final double kSwerveRotationD0 = 5.0;//5.0

    //beginning of auto
    public static final double kSwerveRotationP1 = 8; //8.0
    public static final double kSwerveRotationI1 = 0.0;
    public static final double kSwerveRotationD1 = 200.0;
    
    //swerve Drive PID
    //position/motion magic
    public static final double kSwerveDriveP0 = 0.18; //0.18
    public static final double kSwerveDriveI0 = 0.0; //0.0
    public static final double kSwerveDriveD0 = 3.6; //3.6


    //velocity
    public static final double kSwerveDriveP1 = 0.11; //0.11
    public static final double kSwerveDriveI1 = 0.0; //0.0
    public static final double kSwerveDriveD1 = 0.0; //0.0
    
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    
    public static class Turret {
        public static final double kMaxControlAngle = 33; //In both positive and negative directions | 220.0
        public static final double kMinControlAngle = -145.32; // -25.0
        
        
        
        public static final double kMaxInitialAngle = 33; // 220.0
        public static final double kMinInitialAngle = -145.32; //-25.0
        
        public static final double kEncoderStartingAngle = -208.2;
        // Absolute position of the magnet 309.25
        public static final double kAngleTolerance = 1.0;
        
        // Ratios
        public static final double kInternalEncToOutputRatio = 116.6667;
        public static final double kEncoderToOutputRatio = 1.0;
        
        //PID
        public static final double kLimelightP = 0.025;
        public static final double kLimelightI = 0.0;
        public static final double kLimelightD = 0.00;

        public static final double kP = 0.3;//.3
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0465; // Theoretical: 0.0465
        
        public static final double kMaxSpeed = 22000.0;//22000
        
        // Turret pose with respect to the robot's center
        public static final double kYOffset = 0;//-2.25
        public static final double kXOffset = 0.0; 

        public static final List<double[]> kVisionRanges = Arrays.asList(
            new double[] {kMinControlAngle, 87.0},
            new double[] {93.0, kMaxControlAngle}
        );
    }
    public static class Pigeon{
        public static final double kPIGEON_YAW = 90;
        public static final double kPIGEON_PITCH = 0;
        public static final double kPIGEON_ROLL = 0;
    }
    
    

    // Top Wheel Treemap
    /*public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kWheelTreeMap = new InterpolatingTreeMap<>();
    static{
        kWheelTreeMap.put(new InterpolatingDouble(-500.0), new InterpolatingDouble(0.051)); //0.041 0.079
        kWheelTreeMap.put(new InterpolatingDouble(500.0), new InterpolatingDouble(0.051)); //0.041 0.079
        kWheelTreeMap.put(new InterpolatingDouble(1500.0), new InterpolatingDouble(0.048)); //0.042 0.08
        kWheelTreeMap.put(new InterpolatingDouble(3000.0), new InterpolatingDouble(0.048)); //0.044 0.082
        kWheelTreeMap.put(new InterpolatingDouble(5000.0), new InterpolatingDouble(0.048)); //0.0445 0.0825
    }*/
    
    
    /**
    * First double is distance and second double is the rpm for the bottom wheel
    */
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kWheelRPMToDistanceTreeMap = new InterpolatingTreeMap<>();

    static {
        kWheelRPMToDistanceTreeMap.put(new InterpolatingDouble(83.56), new InterpolatingDouble(3700.0));
        kWheelRPMToDistanceTreeMap.put(new InterpolatingDouble(94.70), new InterpolatingDouble(4250.0));
        kWheelRPMToDistanceTreeMap.put(new InterpolatingDouble(106.60), new InterpolatingDouble(4600.0));
        kWheelRPMToDistanceTreeMap.put(new InterpolatingDouble(115.50), new InterpolatingDouble(4650.0));
        kWheelRPMToDistanceTreeMap.put(new InterpolatingDouble(124.40), new InterpolatingDouble(4650.0));
        kWheelRPMToDistanceTreeMap.put(new InterpolatingDouble(128.40), new InterpolatingDouble(4650.0));
        kWheelRPMToDistanceTreeMap.put(new InterpolatingDouble(132.40), new InterpolatingDouble(4675.0));
        kWheelRPMToDistanceTreeMap.put(new InterpolatingDouble(136.40), new InterpolatingDouble(4700.0));
        kWheelRPMToDistanceTreeMap.put(new InterpolatingDouble(140.40), new InterpolatingDouble(4720.0));
        kWheelRPMToDistanceTreeMap.put(new InterpolatingDouble(145.20), new InterpolatingDouble(5100.0));
    }

    public static class Shooter{
        public static double kShooterEncToOutputRatio = 1.0; //even gear ratio for now
        public static final double kShooterRamp = 0.25; //2.0
        
        //Shooter RPM 
        public static final double kCloseTopRPM = 875.0;
        public static final double kCloseBottomRPM = 2200.0;

        public static final double kMidTopRPM = 1150.0; //1100, 3300, 55 deg
        public static final double kMidBottomRPM = 3100.0; //1500, 3300, 53.5 deg

        public static final double kFarTopRPM = 1135.0; //1400, 3600, 63 deg
        public static final double kFarBottomRPM = 4000.0; //1700, 3400, 61 deg

        public static final double kCornerTopRPM = 1135.0;
        public static final double kCornerBottomRPM = 4000.0;
        
        /**
        * Spin Up
        */
        public static final double kP = 0.025; // WCP Single 4in: 0.02
        public static final double kI = 0.00012; // WCP Single 4in: 0.0001
        public static final double kD = 0.04;
        public static final double kF = 0.048;// 0.09 for 3000 rpm 

    
        /**
        * Hold
        */
        public static final double kHoldP = 0.030; // WCP Single 4in: 0.4
        public static final double kHoldI = 0.00012;
        public static final double kHoldD = 0.240; // WCP Single 4in: 6.38
        public static final double kHoldF = 0.0820; // Value gets changed as soon as it enters hold state so value never gets used
    
        
        public static final double kShooterRPMTolerance = 50.0;
    }
    
    public static class Intake{
        
        public static final double kIntakeP = 1.0;
        public static final double kIntakeI = 0.0;
        public static final double kIntakeD = 0.0;
        public static final double kIntakeF = 0.0;
        
        public static final double kIntakeSpeed = 0.8;
        public static final double kOuttakeSpeed = -0.4;
        public static final double kFeedingSpeed = 0.6;
        public static final double kHumanLoadSpeed = 0.5;
        public static final double kFastIntakeSpeed = 0.75;
    }
    
    public static class Feeder {
        public static final double kAccelSpeed = 0.40;
        public static final double kFeedingSpeed = 0.5;
        public static final double kReverseSpeed = -0.2;
        public static final double kReceivingSpeed = 0.3;
        public static final double kFeederCurrentLimit = 35;
        public static final double kFeedingTime = 0.25;
        public static final SupplyCurrentLimitConfiguration currentLimitConfig = new SupplyCurrentLimitConfiguration(true, Constants.Feeder.kFeederCurrentLimit, Constants.Feeder.kFeederCurrentLimit, 10);
        
        public static final double kRamp = 0.125;
        public static final double kStagingSpeed = 0.15;
        public static final double kFeedingTimeSecond = 3.0;
    }

    public static class AccelWheel {
        public static final double kAccelPercentSpeed = 0.40;
        public static final double kAccelRPM = 0.0;
        public static final double kAccelWheelCurrentLimit = 35;

        public static final double kP = 0.1;
        public static final double kI = 0.00015;//00015
        public static final double kD = 0.02;
        public static final double kF = 0.052;

        public static final double kHoldP = 0.070; 
        public static final double kHoldI = 0.00015;
        public static final double kHoldD = 0.472; 
        public static final double kHoldF = 0.0820;

        public static final double ramp = .05;

        public static final double encoderToOutputRatio = 1;
    
        public static final SupplyCurrentLimitConfiguration currentLimitConfig = new SupplyCurrentLimitConfiguration(true, Constants.AccelWheel.kAccelWheelCurrentLimit, Constants.AccelWheel.kAccelWheelCurrentLimit, 10);
        
        public static final double kRamp = 0.125;
        public static final double kReverseSpeed = 0.45;
    }

    public static class ActuatingHood {
        public static final double kMinHoodAngle = 5.0;
        public static final double kMaxHoodAngle = 11;

        public static final double kDeltaAngle = kMaxHoodAngle - kMinHoodAngle;
        public static final double kMaxLength = 1.968;
        public static final double kMinLength = 0.0;
        public static final double kLengthToAngle = 180.0 / (kMaxLength-kMinLength);
        public static final double kFarRangeThreshold = 80;
    }

    public static class Climber{

        public static final double kMaxControlHeight = 20;
        public static final double kManualSpeed = 0.4;
        public static double kRotationToInches = 3.1415926535/10.0;
        public static double kP = 0.3;//0.3
        public static double kI = 0.0; 
        public static double kD = 0.0; 
        public static double kF = 0.04;//.4

        
        public static double kPTwo = 0.3;//0.3
        public static double kITwo = 0.0;
        public static double kDTwo = 0.0; 
        public static double kFTwo = 0.00;//.4

        public static double kMaxSpeed = 2048 * 10;
        public static double kMaxDescendSpeed = 2048 * 3;
        public static double kEncoderReduction = 10.0;
        public static double kMinControlHeight = 0.0;
        public static double kMaxInitialHeight = 20;
        public static double kEncoderStartingAngle = 0.0;
        public static double kMinInitialHeight = 0.0;
        public static double kHeightTolerance = 0.3;
        

    }
    


}
