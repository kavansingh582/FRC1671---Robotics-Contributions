/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1671.frc;

/**
 * 
 */
public class Settings {

    private static Settings instance = new Settings();

    public static final boolean kIsUsingCompBot = true;
	public static final boolean kIsUsingTractionWheels = true;

    public static final boolean kSimulate = false;
	public static final boolean kResetTalons = false;
    
    // Separate debugging output into the different subsystems so as to not 
    // overload the NetworkTables
    private boolean kDebugSwerve = true;
    private boolean kDebugTurret = false;
    private boolean kDebugShooter = false;
    private boolean kDebugVision = false;
    private boolean kDebugFeeder = false;
    private boolean kDebugHanger = false;

    private boolean kDebugHood= false;
    private boolean kdebugAccelWheels= true;



    private void updateSettings() {
        instance.kDebugSwerve = kDebugSwerve;
        instance.kDebugTurret = kDebugTurret;
        instance.kDebugShooter = kDebugShooter;
        instance.kDebugVision = kDebugVision;
        instance.kDebugFeeder = kDebugFeeder;
        instance.kDebugHanger = kDebugHanger;
        instance.kDebugHood = kDebugHood;
        instance.kdebugAccelWheels = kdebugAccelWheels;
    }


    public static void update() {
        instance.updateSettings();
    }

    public static boolean debugSwerve(){ return instance.kDebugSwerve; }
    public static boolean debugTurret(){ return instance.kDebugTurret; }
    public static boolean debugShooter(){ return instance.kDebugShooter; }
    public static boolean debugVision(){ return instance.kDebugVision; }
    public static boolean debugFeeder(){ return instance.kDebugFeeder ;}
    public static boolean debugHanger(){ return instance.kDebugHanger ;}
    public static boolean debugHood(){ return instance.kDebugHood ;}
    public static boolean debugAccelWheels(){ return instance.kdebugAccelWheels ;}

}
