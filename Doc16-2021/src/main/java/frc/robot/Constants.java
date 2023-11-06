package frc.robot;

import java.util.Arrays;
import java.util.List;

import trajectory_lib.Waypoint;

public class Constants {
    //LIMELIGHT KMS: 10.16.71.11

    public static final double LOOPER_DT = 0.005; //loop time
    public static final int LONG_CAN_TIMEOUT_MS = 100;
    public static final double ENCODER_PULSE_PER_REV = 1440.;
    public static final double FALCON_PULSE_PER_REV = 2048.;
    public static final double ABS_TO_DEG = 360./4096.;

    public static final int WHEEL_DIAMETER = 6;
    public static final double WHEEL_BASE = 32.125; //ft front to back
//drive
    public static final int LEFT_FRONT = 10;
    public static final int LEFT_REAR = 8;
    public static final int RIGHT_FRONT = 6;
    public static final int RIGHT_REAR = 12;

    public static final int DRIVE_IMU = 2;

//Flywheel
    public static final int RIGHT_FLYWHEEL = 4;
    public static final int LEFT_FLYWHEEL = 3;
    public static final int ACCELERATOR_WHEEL = 5;

    public static final double kShooterP = 0.0;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;
    public static final double kShooterF = 0.0;


//turret
    public static final int TURNTABLE_MOTOR = 9;
    public static final double TURNTABLE_KP = 0.04;
    public static final double TURNTABLE_KI = 0.;
    public static final double TURNTABLE_KD = 0.;
    public static final double MIN_OUTPUT = -1.0;
    public static final double MAX_OUTPUT = 1.0;

//intake
    public static final int INTAKE_BAR_MOTOR = 15;
    public static final int HOPPER_MOTOR = 7;
    public static final int BELT_MOTOR = 11;
    public static final int INTAKE_SOL_1 = 0;
    public static final int INTAKE_SOL_2 = 0;

//hood
    public static final int HOOD_SOL_1 = 0;
    public static final int HOOD_SOL_2 = 0;
    public static final int COMPRESSOR = 0;

//control pannel
    public static final int CONTROL_PANNEL_MOTOR = 0;
    public static final int CONTROL_PANNEL_FOLLOWER = 0;

//AGGG
    public static final int AGGG_LEFT = 0;
    public static final int AGGG_RIGHT = 0;
    
    public static final int AGGG_LEFT_ENCODER_1 = 0;
    public static final int AGGG_LEFT_ENCODER_2 = 0;
    public static final int AGGG_RIGHT_ENCODER_1 = 0;
    public static final int AGGG_RIGHT_ENCODER_2 = 0;


//pathfollowing
    public static final double DRIVE_ALLOWED_SPEED = 13.2344; //in ft/sec
    public static final double DRIVE_MAX_ACC = 10.; //in ft/sec^2 ?
    public static final double DRIVE_MAX_JERK = 60.;
    public static final double TEST_ALLOWED_SPEED = 13.2344; //in ft/sec
    public static final double TRAJECTORY_FOLLOWER_KP = 0.04; //proportional gain
    public static final double TRAJECTORY_FOLLOWER_KI = 0.0; //Integral gain
    public static final double TRAJECTORY_FOLLOWER_KD = 0.0; //derivative gain
    public static final double TRAJECTORY_FOLLOWER_KV = 0.0035; //vel if something needs to happen leter change this extend it
    public static final double TRAJECTORY_FOLLOWER_KA = 0.0001; //acc
    public static final double TRAJECTORY_FOLLOWER_KW = 0.0; //rotational PID value

    public static final double HEADING_CONTROLLER_KP = 0.0;
    public static final double HEADING_CONTROLLER_KI = 0.0;
    public static final double HEADING_CONTROLLER_KD = 0.;

//paths feet
    public static final List<Waypoint> rightPath = Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint(15.2, 0.0, 0.0));
    public static final List<Waypoint> centerPath = Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint(5., 0.0, 0.0));
    public static final List<Waypoint> leftPath = Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint(10.5, 0.0, 0.0));
    public static final List<Waypoint> moveOffLinePath = Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint(5., 0.0, 0.0));

}