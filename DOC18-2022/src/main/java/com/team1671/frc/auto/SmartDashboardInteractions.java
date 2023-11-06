package com.team1671.frc.auto;

import com.team1671.frc.auto.modes.StandStillMode;
import com.team1671.frc.auto.modes.DriveAPathMode;
import com.team1671.frc.auto.modes.DriveAPathBackwardsMode;
import com.team1671.frc.auto.modes.QuartileOne_FiveBallToHumanPlayer;
import com.team1671.frc.auto.modes.QuartileOne_ThreeBallAuto;
import com.team1671.frc.auto.modes.QuartileTwo_TwoBallAuto;
import com.team1671.frc.auto.modes.ShootAndMove;
import com.team1671.frc.auto.modes.QuartileThree_FiveBallToHumanPlayer;
import com.team1671.frc.auto.modes.QuartileThree_ThreeBallAuto;
import com.team1671.frc.auto.modes.QuartileFour_TwoBallAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInteractions {
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    
    private static final AutoOption DEFAULT_MODE = AutoOption.STAND_STILL;

    private SendableChooser<AutoOption> modeChooser;

    
    public void initWithDefaults(){
    	modeChooser = new SendableChooser<AutoOption>();
        modeChooser.setDefaultOption(DEFAULT_MODE.name, DEFAULT_MODE);
        modeChooser.addOption(AutoOption.DRIVE_A_PATH_MODE.name, AutoOption.DRIVE_A_PATH_MODE);
        modeChooser.addOption(AutoOption.DRIVE_A_PATH_BACKWARD_MODE.name, AutoOption.DRIVE_A_PATH_BACKWARD_MODE);
        modeChooser.addOption(AutoOption.SHOOTANDMOVE.name, AutoOption.SHOOTANDMOVE);
        modeChooser.addOption(AutoOption.QUARTILEONE_THREEBALL.name, AutoOption.QUARTILEONE_THREEBALL);
        modeChooser.addOption(AutoOption.QUARTILETWO_TWOBALL.name, AutoOption.QUARTILETWO_TWOBALL);
        modeChooser.addOption(AutoOption.QUARTILEONE_FIVEBALL.name, AutoOption.QUARTILEONE_FIVEBALL);
        modeChooser.addOption(AutoOption.QUARTILETHREE_THREEBALL.name, AutoOption.QUARTILETHREE_THREEBALL);
        modeChooser.addOption(AutoOption.QUARTILEFOUR_TWOBALL.name, AutoOption.QUARTILEFOUR_TWOBALL);
        modeChooser.addOption(AutoOption.QUARTILETHREE_FIVEBALL.name, AutoOption.QUARTILETHREE_FIVEBALL);


        SmartDashboard.putData("Mode Chooser", modeChooser);
    	SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
    }
    
    public AutoModeBase getSelectedAutoMode(){
        AutoOption selectedOption = (AutoOption) modeChooser.getSelected();                
        return createAutoMode(selectedOption);
    }
    
    public String getSelectedMode(){
    	AutoOption option = (AutoOption) modeChooser.getSelected();
    	return option.name;
    }

    enum AutoOption{
        STAND_STILL("Stand Still"),
        DRIVE_A_PATH_MODE("Just Drive a Path"),
        DRIVE_A_PATH_BACKWARD_MODE("Drive a Path Backwards"),
        SHOOTANDMOVE("Shoot and Move off Line"),
        QUARTILEONE_THREEBALL("Q1_3 Ball Shots"),
        QUARTILETWO_TWOBALL("Q2_2 Ball Shots"),
        QUARTILEONE_FIVEBALL("Q1_5 Ball Shots"),
        QUARTILETHREE_THREEBALL("Q3_3 Ball Shots"),
        QUARTILEFOUR_TWOBALL("Q4_2 Ball Shots"),
        QUARTILETHREE_FIVEBALL("Q3_5 Ball Shots");
        
    	
    	public final String name;
    	
    	AutoOption(String name){
    		this.name = name;
    	}
    }

    private AutoModeBase createAutoMode(AutoOption option){
    	switch(option){
            case STAND_STILL:
                return new StandStillMode();
            case DRIVE_A_PATH_MODE:
                return new DriveAPathMode();
            case DRIVE_A_PATH_BACKWARD_MODE:
                return new DriveAPathBackwardsMode();
            case SHOOTANDMOVE:
                return new ShootAndMove();            
            case QUARTILEONE_THREEBALL:
                return new QuartileOne_ThreeBallAuto();
            case QUARTILETWO_TWOBALL:
                return new QuartileTwo_TwoBallAuto();
            case QUARTILEONE_FIVEBALL:
                return new QuartileOne_FiveBallToHumanPlayer();
            case QUARTILETHREE_THREEBALL:
                return new QuartileThree_ThreeBallAuto();
            case QUARTILEFOUR_TWOBALL:
                return new QuartileFour_TwoBallAuto();
            case QUARTILETHREE_FIVEBALL:
                return new QuartileThree_FiveBallToHumanPlayer();
            default:
                System.out.println("ERROR: unexpected auto mode: " + option);
                return new StandStillMode();
    	}
    }
    
    public void output(){
    	SmartDashboard.putString(SELECTED_AUTO_MODE, getSelectedMode());
    }
}
