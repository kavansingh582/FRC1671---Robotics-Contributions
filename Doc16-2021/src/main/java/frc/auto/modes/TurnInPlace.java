package frc.auto.modes;


import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.TurnInPlaceAction;

public class TurnInPlace extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new TurnInPlaceAction(90.0, true, 2.0, 20.0));
        
    }
    
}