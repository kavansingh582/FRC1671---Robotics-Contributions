package frc.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;



public class LED{
    public AddressableLED led;
    public AddressableLEDBuffer ledBuffer;
    public DigitalInput bannerSensor;

    public double timestamp = Timer.getFPGATimestamp();
    public double time;
    private static LED instance;
    private int count;


    public static LED getInstance() {
        if(instance == null) {
            instance = new LED();
        }
        return instance;
    }

    public LED(){
        led = new AddressableLED(0); // fix the port number and contact electronics <---------- Remember
        ledBuffer = new AddressableLEDBuffer(60);
        bannerSensor = new DigitalInput(0); // fix the port number and contact electronics <---------- Remember
     
        
        //led.setLength(ledBuffer.getLength());
    
        // Set the data
        //led.setData(ledBuffer);
        //led.start();
    }


    //fix this it will not work below
    public int countBall() {
        if(bannerSensor.get() == true){
            count = 1 ;   
        }
        if(bannerSensor.get() == true){
            count = 2 ;       
        } 
        if(bannerSensor.get() == true){
            count = 3 ;       
         } 
        if(bannerSensor.get() == true){
            count = 4 ;       
        } 
        if(bannerSensor.get() == true){
            count = 5;       
        }     
        return count;
    }

    public void countSwitch(){
        if(count == 0){
            red();
        }
        if(count == 1){
            yellow();
        }
        if(count == 2){
            yellow();
        }
        if(count == 3){
            yellow();
        }
        if(count == 4){
            yellow();
        }
        if(count == 5){
            green();
        }

    }

    public void blue() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for blue
            ledBuffer.setRGB(i, 0, 0, 255);
            led.setData(ledBuffer);
        }

    }

    public void red() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            ledBuffer.setRGB(i, 255, 0, 0);
            led.setData(ledBuffer);
        }
        
    }

    public void green() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for green
            ledBuffer.setRGB(i, 0, 255, 0);
            led.setData(ledBuffer);
        }
        
    }

    public void purple() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for green
            ledBuffer.setRGB(i, 106, 13, 173);
            led.setData(ledBuffer);
        }
        
    }

    public void yellow() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for green
            ledBuffer.setRGB(i, 255, 255, 0);
            led.setData(ledBuffer);
        }
        
    }

    public void orange() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for green
            ledBuffer.setRGB(i, 255, 165, 0);
            led.setData(ledBuffer);
        }
        
    }
    public void pink() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for green
            ledBuffer.setRGB(i, 255, 20, 147);
            led.setData(ledBuffer);
        }
        
    }
    public void turquoise() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for green
            ledBuffer.setRGB(i, 64, 224, 208);
            led.setData(ledBuffer);
        }
        
    }
    
    public void white() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for green
            ledBuffer.setRGB(i, 64, 224, 208);
            led.setData(ledBuffer);
        }
        
    }

    public void Ethancolorchange(double time2) {
        time2 = Timer.getFPGATimestamp();
        if(time % 6 <= 2){
            red();
        }
        else if (time % 6 <= 4){
            blue();
        }
        else if (time % 6 <= 6){
            white();
        }    

    }
  
    public void colorCycle(double time){
        time = Timer.getFPGATimestamp();
            if(time % 16 <= 2){
                red();
            }
            else if (time % 16 <= 4){
                orange(); 
            }
            else if (time % 16 <= 6 ){
                yellow();
            }
            else if (time % 16 <= 8){
                green();
            }
            else if (time % 16 <= 10){
                blue();
            }
            else if (time % 16 <= 12 ){
                pink();
            }
            else if (time % 16 <= 14){
                purple(); 
            }
            else if (time % 16 <= 16){
                turquoise();
            
        } 
      
    }
    
}






