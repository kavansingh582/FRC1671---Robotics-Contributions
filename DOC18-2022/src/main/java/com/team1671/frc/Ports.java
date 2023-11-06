package com.team1671.frc;


public class Ports {

    // CAN Devices
    // Falcons
    public static final int FRONT_RIGHT_ROTATION= 11;
    public static final int FRONT_RIGHT_DRIVE   = 15;
    public static final int FRONT_LEFT_ROTATION = 8;
    public static final int FRONT_LEFT_DRIVE    = 12;
    public static final int REAR_LEFT_ROTATION  = 7;
    public static final int REAR_LEFT_DRIVE     = 3;
    public static final int REAR_RIGHT_ROTATION = 4;
    public static final int REAR_RIGHT_DRIVE    = 0;
    public static final int INTAKE_MOTOR = 9;
    public static final int MASTER_FLYWHEEL = 14;
	public static final int SLAVE_FLYWHEEL = 13;
    public static final int FEEDER = 6;
    public static final int TURRET = 10;
    public static final int CLIMBER_SLAVE = 5;
    public static final int CLIMBER_MASTER = 2; 
    public static final int ACCELWHEEL = 1;
  



    // MISC CAN
    public static final int PIGEON = 16;
    public static final int CANDLE = 17;


    //Digital Inputs
    public static final int FRONT_RIGHT_ENCODER = 0;
    public static final int FRONT_LEFT_ENCODER = 1;
    public static final int REAR_LEFT_ENCODER = 2;
    public static final int REAR_RIGHT_ENCODER = 3;
    public static final int TURRET_ENCODER = 4;
    public static final int[] kModuleEncoders = new int[]{FRONT_RIGHT_ENCODER, FRONT_LEFT_ENCODER,
        REAR_LEFT_ENCODER, REAR_RIGHT_ENCODER};
    public static final int FEEDER_BANNER1 = 5;
    public static final int FEEDER_BANNER2 = 6;
    
    
    //PWM
    public static final int HOOD1 = 0;
    public static final int HOOD2 = 1; 


    // Pneumatics
    public static final int PCM = 0;
    public static final int INTAKE_EXTENDER = 0;
    public static final int CLIMBER_EXTENDER = 1;
    public static final int MINICLIMBERPISTON = 2;
   















    }
