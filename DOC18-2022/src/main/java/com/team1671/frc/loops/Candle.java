package com.team1671.frc.loops;



//imports
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.team1671.frc.Ports;
import com.team1671.frc.Constants;
import com.team1671.frc.subsystems.Feeder;
import com.team1671.frc.subsystems.Shooter;
import com.team1671.frc.subsystems.Swerve;
import com.team1671.frc.subsystems.Turret;
import com.team1671.frc.subsystems.Turret.State;
import com.team1671.frc.subsystems.requests.Request;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Candle implements Loop {
    static Candle instacne = new Candle();
    public Swerve drive;
	public Turret turret;
    public Shooter shooter;
    public Feeder feeder;

    // create an instance of the candle
    public static Candle instance = null;
    public static Candle getInstance(){
        if (instance == null){
            instance = new Candle();
        }
        return instance;
    }

    //all possible candle states
    public enum State{
        IDLE,
        RGB,
        LENGTH,
        SOFTSTROBE,
        ULTRASTROBE,
        CHASE,
        STRIPES;
    
        State(){
        }
    }

    public class Stripe{
        public int length;
        public double start;
        public double speed;
        public int red;
        public int green;
        public int blue;
        public int validAreaStart;
        public int validAreaEnd;
        public int lineBeginning;
        public String id;

        public Stripe(int lengthOStipe, int startOStripe, Double speedOStripe, int redOStripe, int greenOStripe, int blueOStripe, int validAreaStartOStripe, int validAreaEndOStripe, String idOstripe){
            length = lengthOStipe;
            start = startOStripe;
            speed = speedOStripe;
            red = redOStripe;
            green = greenOStripe;
            blue = blueOStripe;
            validAreaStart = validAreaStartOStripe;
            validAreaEnd = validAreaEndOStripe;
            id = idOstripe;
            start = Math.max(validAreaStart, start);
        }

        
        public void updateC1(){
            candle.setLEDs(0, 0, 0, 0, this.lineBeginning+8, Math.min(this.validAreaEnd-this.lineBeginning+1,this.length));
        }
        public void updateC2(){
            candle.setLEDs(0, 0, 0, 0, this.validAreaStart+8, Math.max(0, this.lineBeginning+this.length-this.validAreaEnd-1));
        }
        public void update1(){
            start=start+speed;
            lineBeginning = (int)Math.round(start);
            if(start>validAreaEnd){
                start=start+validAreaStart-validAreaEnd-1;
            }
            if(lineBeginning>validAreaEnd){
                lineBeginning=lineBeginning+validAreaStart-validAreaEnd-1;
            }
            if(start<validAreaStart){
                start=start+(validAreaEnd-validAreaStart)+1;
            }
            if(lineBeginning<validAreaStart){
                lineBeginning=lineBeginning+(validAreaEnd-validAreaStart)+1;
            }
            candle.setLEDs(this.red, this.green, this.blue, 255, this.lineBeginning+8, Math.min(this.validAreaEnd-this.lineBeginning+1,this.length));
        }
        public void update2(){
            candle.setLEDs(this.red, this.green, this.blue, 255, this.validAreaStart+8, Math.max(0, this.lineBeginning+this.length-this.validAreaEnd-1));
        }
    }
    //define RGB and set default value
    private int red;
    private int green;
    private int blue;

    private boolean strobeDir = true;
    private int softStrobeFactor = 30;
    private int softStrobeMax= 30;
    private int softStrobeMin= 3;
    private int ultraStrobeFactor = 10;
    private int ultraStrobeMax= 10;
    private int ultraStrobeMin= 0;

    Stripe chase = new Stripe(8, 0, 2.0, red, green, blue, 0, 92,"Chase");
    private int chaseCycle = 0;

    private int lengthModeLength=4;
    private int lengthModeStart=0;
    private boolean[] errorCheckList;

    private double prevUpdateTimestamp = 0;
    private double prevErrorCheckTimestamp = 0;


    private Stripe[] stripes = new Stripe[0];

    private boolean doneStripes = true;
    private int errorCheckNum = 0;
    private int stripeNum = 0;
    private int stripePhase = 1;

    private boolean climberEnabled = false;
    private boolean isIntaking = false;

    private boolean needDeletion = false;
    private boolean isMarkedForDeletion;
    private String[] needDeletionIds;

    //creating candle and state
    private CANdle candle;
    private State currentState = State.IDLE;


    //defining the properties of Candle
    public Candle(){
        drive = Swerve.getInstance();
		turret = Turret.getInstance();
        shooter = Shooter.getInstance();
        feeder = Feeder.getInstance();
        candle = new CANdle(Ports.CANDLE);
    }

     
    public void turnOnDefaultLED(){
        createStripe(5, 0, (double)(22/15), 0, 255, 255, 0, 21,"Default 1");
        createStripe(5, 22, (double)(20/15), 255, 0, 0, 22, 42,"Default 2");
        createStripe(5, 43, (double)(27/15), 0, 255, 255, 43, 70,"Default 3");
    }
    public void climberCandle(){
        createStripe(5, 0, (double)(22/15), 150, 0, 0, 0, 21,"Climbing");
        createStripe(5, 22, (double)(20/15), 0, 0, 150, 22, 42,"Climbing");
        createStripe(5, 43, (double)(27/15), 0, 150, 0, 43, 70,"Climbing");
        createStripe(5, 71, (double)(22/15), 0, 0, 150, 71, 92,"Climbing");
    }

     //sets the entire led to RGB
    public void lightOnRGB(int R, int G, int B){
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        candle.configAllSettings(config);

        candle.setLEDs(R, G, B);
    }

    //Light up specific length of leds at a specific point with RGB
    public void lightOnRGBLength(int R, int G, int B, int startID, int length){
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        candle.setLEDs(R, G, B, 255, startID, length);
        
    }
    /*public Request setClimberEnabled(boolean in){
        return new Request(){

            @Override
            public void act(){
                climberEnabled = in;
            }
        };
    }*/
    public void setClimberEnabled(boolean in){
                climberEnabled = in;
    }
    

    /*public Request setIntaking(boolean isRobotIntaking){
        return new Request(){

            @Override
            public void act(){
                isIntaking = isRobotIntaking;
            }
        };
    }*/
    public void setIntaking(boolean isRobotIntaking){
                isIntaking = isRobotIntaking;
    }
    public void clearAll(){
        candle.setLEDs(0,0,0);
    }
    public void deleteStripe(String deletionId){
        addStripeToDeletion(deletionId);
    }
    public void deleteAllStripes(){
        candle.setLEDs(0, 0, 0, 0, 8, 71);
        stripes = new Stripe[0];
    }
    public void createStripe(int lengthOStipe, int startOStripe, Double speedOStripe, int redOStripe, int greenOStripe, int blueOStripe, int validAreaStartOStripe, int validAreaEndOStripe, String idOstripe){
        Stripe newStripe = new Stripe(lengthOStipe, startOStripe, speedOStripe, redOStripe, greenOStripe, blueOStripe, validAreaStartOStripe, validAreaEndOStripe,idOstripe);
        stripes = addStripeToArray(newStripe);
    }

    public void softStrobeLED(){
        if(strobeDir){
            softStrobeFactor--;
        }else{
            softStrobeFactor++;
        }
        if(softStrobeFactor==softStrobeMin){
            strobeDir = false;
        }else if(softStrobeFactor==softStrobeMax){
            strobeDir =true;
        }
        //System.out.println((int)( red/softStrobeMax * softStrobeFactor) + (int)(green/softStrobeMax * softStrobeFactor) + (int)(blue/ softStrobeMax * softStrobeFactor));
        lightOnRGBLength((int)(red/softStrobeMax * softStrobeFactor), (int)(green/softStrobeMax * softStrobeFactor), (int)(blue/softStrobeMax * softStrobeFactor),8,92);
    }
    public void ultraStrobeLED(){
        if(strobeDir){
            ultraStrobeFactor--;
        }else{
            ultraStrobeFactor++;
        }
        if(ultraStrobeFactor==ultraStrobeMin){
            strobeDir = false;
        }else if(ultraStrobeFactor==ultraStrobeMax){
            strobeDir =true;
        }
        //System.out.println((int)( red/ultraStrobeMax * ultraStrobeFactor) + (int)(green/ultraStrobeMax * ultraStrobeFactor) + (int)(blue/ ultraStrobeMax * ultraStrobeFactor));
        lightOnRGBLength((int)(red/ultraStrobeMax * ultraStrobeFactor), (int)(green/ultraStrobeMax * ultraStrobeFactor), (int)(blue/ultraStrobeMax * ultraStrobeFactor),8,92);
    }

    public void setChaseStripe(){
        chase.red=red;
        chase.green=green;
        chase.blue=blue;
        switch(chaseCycle){
            case 0:
                lightOnRGBLength(0,0,0,8,92);
                chaseCycle=1;
            break;
            case 1:
                chase.update1();
                chaseCycle=2;
            break;
            case 2:
                chase.update2();
                chaseCycle=0;
            break;

        }

    }

    //set RGB valuse
    public void setRGB(int R, int G, int B){
        red=R;
        green=G;
        blue=B;
    }

    //set individual RGB values
    public void setR(int R){
        red=R;
    }
    public void setG(int G){
        green=G;
    }
    public void setB(int B){
        blue=B;
    }


    

    //get RGB values
    public int getR(){
        return red;
    }
    public int getG(){
        return green;
    }
    public int getB(){
        return blue;
    }

    public boolean[] getFunctional() {
		boolean[] notBrokenList = new boolean[8];
		notBrokenList[0] = drive.isFunctional()[0];
		notBrokenList[1] = drive.isFunctional()[1];
		notBrokenList[2] = drive.isFunctional()[2];
		notBrokenList[3] = drive.isFunctional()[3];
		notBrokenList[4] = turret.isEncoderConnected();
		notBrokenList[5] = true;
		notBrokenList[6] = true;
		notBrokenList[7] = true;
		return notBrokenList;
	}


    public Stripe[] addStripeToArray(Stripe newStripe){
        int n = stripes.length;
        // create a new array of size n+1
        Stripe newarr[] = new Stripe[n + 1];
  
        // insert the elements from
        // the old array into the new array
        // insert all elements till n
        // then insert x at n+1
        for (int i = 0; i < n; i++)
            newarr[i] = stripes[i];
  
        newarr[n] = newStripe;
        return newarr;
    }
    public Stripe[] eliminateStripeFromArray(int eliminationNum){
        int n = stripes.length;
        // create a new array of size n+1
        Stripe newarr[] = new Stripe[n - 1];
  
        // insert the elements from
        // the old array into the new array
        // insert all elements till n
        // then insert x at n+1
        for (int i = 0; i < n-1; i++){
            if(i<eliminationNum){
                newarr[i] = stripes[i];
            }else{
                newarr[i] = stripes[i+1];  
            }
        }
        return newarr;
    }
    public String[] addStripeToDeletion(String newId){
        int n = needDeletionIds.length;
        // create a new array of size n+1
        String newarr[] = new String[n + 1];
  
        // insert the elements from
        // the old array into the new array
        // insert all elements till n
        // then insert x at n+1
        for (int i = 0; i < n; i++)
            newarr[i] = needDeletionIds[i];
  
        newarr[n] = newId;
        return newarr;
    }

    /**
     * Sets current candle state to the state you pass
     * @param newState The state you want; comes from the enum of states
     */
    public void setState(State newState){
        if(currentState != newState){
            currentState = newState;
            System.out.println("state change for Candle");
            stripes = new Stripe[0];
            needDeletionIds = new String[0];
            if(currentState!=State.STRIPES){
                doneStripes=true;
            }
        }else{
        
        }
        clearAll();
    }

    /**
     * Changes the state of the candle to desiredState using setState()
     * @param desiredState The state you want the candle to be in
     */
    public void conformToState(State desiredState){
        setState(desiredState);
    }


     /**
     * Returns the candle's current state
     */
    public State getstate(){
        return currentState;
    }



    public void stateChange(State desiredState, int r, int g, int b){
        conformToState(desiredState); 
        red=r;
        green=g;
        blue=b;
}

     /**
     * Changes your candle state 
     * @param desiredState wanted state
     * @param r red value
     * @param g green value
     * @param b blue value
     * @return A request
     */
    public Request stateChangeRequest(State desiredState, int r, int g, int b){
        return new Request(){

            @Override
            public void act(){
                conformToState(desiredState); 
                red=r;
                green=g;
                blue=b;
                if(currentState!=State.STRIPES){
                    doneStripes=true;
                }
            }
        };
    }
    public Request stateChangeRequest(State desiredState){
        return new Request(){

            @Override
            public void act(){
                stateChangeRequest(desiredState,0,0,0);
            }
        };
    }
    public Request defualtLEDRequest(){
        return new Request(){

            @Override
            public void act(){
                turnOnDefaultLED();
            }
        };
    }
    public Request climberLEDRequest(){
        return new Request(){

            @Override
            public void act(){
                climberCandle();
            }
        };
    }


    /*
    * 
    *
    *
    */
    public Request createStripeRequest(int lengthOStipe, int startOStripe, Double speedOStripe, int redOStripe, int greenOStripe, int blueOStripe, int validAreaStartOStripe, int validAreaEndOStripe,String idOstripe){
        return new Request(){

            @Override
            public void act(){
                createStripe(lengthOStipe, startOStripe, speedOStripe, redOStripe, greenOStripe, blueOStripe, validAreaStartOStripe, validAreaEndOStripe,idOstripe);
            }
        };
    }
    public Request deletionRequest(String deletionId){
        return new Request(){

            @Override
            public void act(){
                deleteStripe(deletionId);;
            }
        };
    }
    public Request setLEDs( int r, int g, int b,int w, int start, int length){
        return new Request(){

            @Override
            public void act(){
                candle.setLEDs(r, g, b, w, start, length);
            }
        };
    }
    
   
    //what you want to do on robot startup
    @Override
    public void onStart(double timestamp) {
        conformToState(State.IDLE);
        red = 0;
        green = 0;
        blue = 0;
        stripes = new Stripe[0];
    }

    @Override
    public void onLoop(double timestamp) {
        //System.out.println(currentState);
        //System.out.println(climberEnabled);
        outputTelemetry();
        if(climberEnabled){
            stateChange(State.STRIPES,0,0,0);
            climberCandle();
        }else if(turret.currentState==Turret.State.CRUDE_VISION&&turret.hasReachedAngle()&&shooter.hasReachedSetpoint()){
            stateChange(State.ULTRASTROBE, 0, 150, 0);
        }else if(shooter.hasReachedSetpoint()){
            stateChange(State.CHASE, 0, 150, 0);
        }else if(feeder.getBanner1()){
            stateChange(Candle.State.ULTRASTROBE,0,150,0);
        }else if(isIntaking){
            stateChange(State.CHASE, 150, 150, 0);
        }else if(feeder.getBallCount()>0){
            stateChange(Candle.State.SOFTSTROBE,0,150,0);
        }else{
            if(DriverStation.getAlliance()== DriverStation.Alliance.Blue){
                stateChange(Candle.State.SOFTSTROBE,0,0,150);
            }else if(DriverStation.getAlliance()== DriverStation.Alliance.Red){
                stateChange(Candle.State.SOFTSTROBE,150,0,0);
            }else{
                stateChange(Candle.State.SOFTSTROBE,150,150,150);
            }
        }
        if(timestamp>prevErrorCheckTimestamp+5){
            errorCheckList = getFunctional();
            if(errorCheckList[errorCheckNum]==true){
                candle.setLEDs(0, 255, 0, 255, errorCheckNum, 1);
            }else{
                candle.setLEDs(255, 0, 0, 255, errorCheckNum, 1);
            }
            errorCheckNum++;
            if(errorCheckNum>=8){
                errorCheckNum = 0;
                prevErrorCheckTimestamp = timestamp;
            }
        }else{
            if(currentState==State.RGB){
                lightOnRGB(red,green,blue);
            }else if(currentState==State.IDLE){
                lightOnRGBLength(150,0,0,8,71);
            }else if(currentState==State.LENGTH){
                lightOnRGBLength(red,green,blue,lengthModeStart,4);
            }else if(currentState==State.SOFTSTROBE){
                softStrobeLED();
            }else if(currentState==State.ULTRASTROBE){
                ultraStrobeLED();
            }else if(currentState==State.CHASE){
                setChaseStripe();
            }else if(currentState==State.STRIPES){
                if(timestamp>prevUpdateTimestamp+1){
                    doneStripes = false;
                    needDeletion = false;
                    stripePhase = 1;
                    stripeNum = 0;
                    prevUpdateTimestamp = timestamp;
                }
                if(doneStripes == false){
                    if(stripeNum<stripes.length){
                        if(stripes[stripeNum].speed==0){
                            stripes[stripeNum].update1();
                            stripeNum++;
                        }else{
                            switch(stripePhase){
                                case 1:
                                    stripes[stripeNum].updateC1();
                                    stripePhase = 2;
                                break;
                                case 2:
                                    stripePhase = 3;
                                    stripes[stripeNum].updateC2();
                                break;
                                case 3:
                                    for(int i=0;i<needDeletionIds.length;i++){
                                        if(stripes[stripeNum].id == needDeletionIds[i]){
                                            isMarkedForDeletion = true;
                                        }
                                    }
                                    if(isMarkedForDeletion){
                                        stripePhase = 1;
                                        needDeletion = true;
                                        eliminateStripeFromArray(stripeNum);
                                        stripeNum++;
                                    }else{
                                        stripes[stripeNum].update1();
                                        stripePhase = 4;
                                    }
                                break;
                                case 4:
                                    stripePhase = 1;
                                    stripes[stripeNum].update2();
                                    stripeNum++;
                                break;
                            }
                        }
                    }else{
                        if(needDeletion == false){
                            needDeletionIds = new String[0];
                        }
                        doneStripes = true;
                    }

                }
            }
        }
    }
    
    //what you want to do on robot end
    @Override
    public void onStop(double timestamp) {
        conformToState(State.IDLE);
        lightOnRGB(0,0,0);
    }    

    
    public void outputTelemetry() {
        SmartDashboard.putString("Candle State", currentState.toString());
    }

    
    public void stop() {
        conformToState(State.IDLE);
    }

    
    /*public void registerEnabledLoops(ILooper enabledLooper){
        //enabledLooper.register(loop);
    }*/

}

