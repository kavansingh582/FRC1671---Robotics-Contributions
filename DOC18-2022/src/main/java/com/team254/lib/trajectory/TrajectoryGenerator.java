package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1671.frc.Constants;
import com.team1671.frc.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.timing.CurvatureVelocityConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.wpilibj.Timer;

public class TrajectoryGenerator {
    //all in in
    private static final double kMaxVelocity = 130.0; //120
    private static final double kMaxAccel = 80.0; // 120 
    private static final double kMaxDecel = 80.0; //72
    private static final double kMaxVoltage = 9.0; //9.0
    
    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;
    
    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }
    
    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }
    
    public void generateTrajectories() {
        if(mTrajectorySet == null) {
            double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }
    
    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }
    
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
    boolean reversed,
    final List<Pose2d> waypoints,
    final List<TimingConstraint<Pose2dWithCurvature>> constraints,
    double max_vel,  // inches/s
    double max_accel,  // inches/s^2
    double max_decel,
    double max_voltage,
    double default_vel,
    int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_decel, max_voltage, 
        default_vel, slowdown_chunks);
    }
    
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
    boolean reversed,
    final List<Pose2d> waypoints,
    final List<TimingConstraint<Pose2dWithCurvature>> constraints,
    double start_vel,  // inches/s
    double end_vel,  // inches/s
    double max_vel,  // inches/s
    double max_accel,  // inches/s^2
    double max_decel,
    double max_voltage,
    double default_vel,
    int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_decel, max_voltage, 
        default_vel, slowdown_chunks);
    }
    
    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the right.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON LEFT! (mirrored about +x axis for RIGHT)
    static final Pose2d quartileOneStartingPose = Constants.quartileOneStartingPose;
    static final Pose2d quartileTwoStartingPose = Constants.quartileTwoStartingPose; //q2
    static final Pose2d quartileThreeStartingPose = Constants.quartileThreeStartingPose;
    static final Pose2d quartileFourStartingPose = Constants.quartileFourStartingPose;
    static final Pose2d testStartPose = Constants.testStartPose;


    //balls 1-4 clockwise including the outlier RED BALLS starting from top of center white tape
    public static final Pose2d redBall1 = new Pose2d(new Translation2d(350.85714285714283, -150.57142857142858), Rotation2d.fromDegrees(0));
    public static final Pose2d redBall2 = new Pose2d(new Translation2d(449.7142857142857, -88.28571428571429), Rotation2d.fromDegrees(0));
    public static final Pose2d redBall3 = new Pose2d(new Translation2d(606.8571428571429, -117.42857142857143), Rotation2d.fromDegrees(0));
    public static final Pose2d redBall4 = new Pose2d(new Translation2d(453.14285714285717, 80.85714285714286), Rotation2d.fromDegrees(0));

    public static final Pose2d blueBall1 = new Pose2d(new Translation2d(298.2857142857143, 149.42857142857144), Rotation2d.fromDegrees(0));
    public static final Pose2d blueBall2 = new Pose2d(new Translation2d(198.85714285714286, 87.14285714285714), Rotation2d.fromDegrees(0));
    public static final Pose2d blueBall3 = new Pose2d(new Translation2d(42.285714285714285, 116.85714285714283), Rotation2d.fromDegrees(0));
    public static final Pose2d blueBall4 = new Pose2d(new Translation2d(195.42857142857142, -81.42857142857143), Rotation2d.fromDegrees(0));


    //static final Pose2d partnerStartingPose = new Pose2d(Constants.kPartnerRobotStartingPose.getTranslation().translateBy(new Translation2d(0.0, 0.0)), Rotation2d.fromDegrees(0.0));

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> left) {
                this.left = left;
                this.right = TrajectoryUtil.mirrorTimed(left, left.defaultVelocity());
            }
            
            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }
            
            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }
        
        //checkpoint paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> Q1ToRedBall1;
        public final Trajectory<TimedState<Pose2dWithCurvature>> RedBall1toBall2;
        public final Trajectory<TimedState<Pose2dWithCurvature>> Q2ToRedBall4;
        public final Trajectory<TimedState<Pose2dWithCurvature>> Q3ToBlueBall1;
        public final Trajectory<TimedState<Pose2dWithCurvature>> BlueBall1toBall2;
        public final Trajectory<TimedState<Pose2dWithCurvature>> Q4ToBlueBall4;
        public final Trajectory<TimedState<Pose2dWithCurvature>> RedBall2toBall3;
        public final Trajectory<TimedState<Pose2dWithCurvature>> BlueBall2toBall3;
        public final Trajectory<TimedState<Pose2dWithCurvature>> RedBall3toBall2;
        public final Trajectory<TimedState<Pose2dWithCurvature>> BlueBall3toBall2;
        //Test Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> testPath;
        public final Trajectory<TimedState<Pose2dWithCurvature>> backTestPath;
        
        //Preliminary Auto Paths
        
        private TrajectorySet() {
            //Test Paths
            testPath = getTestPath();
            backTestPath = getBackTestPath();
            Q1ToRedBall1 = getQ1toRedBall1();
            RedBall1toBall2 = getRedBall1toBall2();
            Q2ToRedBall4 = getQ2toRedBall4();
            Q3ToBlueBall1 = getQ3toBlueBall1();
            BlueBall1toBall2 = getBlueBall1toBall2();
            Q4ToBlueBall4 = getQ4toBlueBall4();
            RedBall2toBall3 = getRedBall2toBall3();
            BlueBall2toBall3 = getBlueBall2toBall3();
            RedBall3toBall2 = getRedBall3toBall2();
            BlueBall3toBall2 = getBlueBall3toBall2();
            //Preliminary Auto Paths
        }
        
        //private double halfWidth = 15;

        
        //test path moves 10 ft.
        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(testStartPose.getTranslation(), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(testStartPose.getTranslation().translateBy(new Translation2d(36. , 0.0)), Rotation2d.fromDegrees(0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        //test path moves -10 ft.
        private Trajectory<TimedState<Pose2dWithCurvature>> getBackTestPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(testStartPose.getTranslation().translateBy(new Translation2d(36. , 0.0)), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(testStartPose.getTranslation(), Rotation2d.fromDegrees(0.0)));
            
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }



        //CHECKPOINT POSITIONS
        //red
        private Trajectory<TimedState<Pose2dWithCurvature>> getQ1toRedBall1(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(quartileOneStartingPose.getTranslation(), Rotation2d.fromDegrees(-160.)));
            waypoints.add(new Pose2d(redBall1.getTranslation(), Rotation2d.fromDegrees(0.0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRedBall1toBall2(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(redBall1.getTranslation(), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(redBall2.getTranslation(), Rotation2d.fromDegrees(45.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getQ2toRedBall4(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(quartileTwoStartingPose));
            waypoints.add(new Pose2d(redBall4.getTranslation(), Rotation2d.fromDegrees(21.)));

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>>getRedBall2toBall3(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(redBall2.getTranslation(), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(redBall3.getTranslation(), Rotation2d.fromDegrees(-45.)));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>>getRedBall3toBall2(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(redBall3.getTranslation(), Rotation2d.fromDegrees(-45.)));
            waypoints.add(new Pose2d(redBall2.getTranslation(), Rotation2d.fromDegrees(0.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }




        //blue
        private Trajectory<TimedState<Pose2dWithCurvature>> getQ3toBlueBall1(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(quartileThreeStartingPose.getTranslation(), Rotation2d.fromDegrees(-110.+180)));
            waypoints.add(new Pose2d(blueBall1.getTranslation(), Rotation2d.fromDegrees(0.0+180)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getBlueBall1toBall2(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(blueBall1.getTranslation(), Rotation2d.fromDegrees(0.0+180)));
            waypoints.add(new Pose2d(blueBall2.getTranslation(), Rotation2d.fromDegrees(45.+180)));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getQ4toBlueBall4(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(quartileFourStartingPose);
            waypoints.add(new Pose2d(blueBall4.getTranslation(), Rotation2d.fromDegrees(21.+180.)));

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>>getBlueBall2toBall3(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(blueBall2.getTranslation(), Rotation2d.fromDegrees(0.0+180)));
            waypoints.add(new Pose2d(blueBall3.getTranslation(), Rotation2d.fromDegrees(-45.+180)));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>>getBlueBall3toBall2(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(blueBall3.getTranslation(), Rotation2d.fromDegrees(-45.+180)));
            waypoints.add(new Pose2d(blueBall2.getTranslation(), Rotation2d.fromDegrees(0.0+180)));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }       
    }
}
