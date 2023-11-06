package com.team1671.frc.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1671.frc.Constants;
import com.team1671.frc.Ports;
import com.team1671.frc.RobotState;
import com.team1671.frc.loops.ILooper;
import com.team1671.frc.loops.LimelightProcessor;
import com.team1671.frc.loops.Candle;
import com.team1671.frc.loops.Loop;
import com.team1671.frc.subsystems.Feeder.State;
import com.team1671.frc.subsystems.requests.LambdaRequest;
import com.team1671.frc.subsystems.requests.ParallelRequest;
import com.team1671.frc.subsystems.requests.Request;
import com.team1671.frc.subsystems.requests.SequentialRequest;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;

public class Superstructure extends Subsystem {

	private Compressor compressor;
	
	public Swerve drive;
	public Turret turret;
	public Intake intake;
	public Feeder feeder;
	public Shooter shooter;
	public Candle candle;
	public Climber climber;
	public ActuatingHood actuatingHood;
	
	public Superstructure(){
		compressor = new Compressor(Ports.PCM, PneumaticsModuleType.CTREPCM);
		
		drive = Swerve.getInstance();
		turret = Turret.getInstance();
		intake = Intake.getInstance();
		feeder = Feeder.getInstance();
		shooter = Shooter.getInstance();
		candle = Candle.getInstance();
		climber = Climber.getInstance();
		actuatingHood = ActuatingHood.getInstance();

		queuedRequests = new ArrayList<>(0);
	}
	private static Superstructure instance = null;
	public static Superstructure getInstance(){
		if(instance == null){
			instance = new Superstructure();
		}
		return instance;
	}

	private Request activeRequest = null;
	private List<Request> queuedRequests = new ArrayList<>();
	
	private boolean newRequest = false;
	private boolean allRequestsCompleted = false;
	public boolean requestsCompleted(){ return allRequestsCompleted; }
	
	private void setActiveRequest(Request request){
		activeRequest = request;
		newRequest = true;
		allRequestsCompleted = false;
	}
	
	private void setQueue(List<Request> requests){
		clearQueue();
		for(Request request : requests) {
			queuedRequests.add(request);
		}
	}

	private void setQueue(Request request) {
		setQueue(Arrays.asList(request));
	}

	private void clearQueue() {
		queuedRequests.clear();
	}
	
	public void request(Request r){
		setActiveRequest(r);
		clearQueue();
	}
	
	public void request(Request active, Request queue){
		setActiveRequest(active);
		setQueue(queue);
	}
	
	public void queue(Request request){
		queuedRequests.add(request);
	}
	
	public void replaceQueue(Request request){
		setQueue(request);
	}
	
	boolean isPrefire = false;

	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			stop();
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized(Superstructure.this) {
				if(newRequest && activeRequest != null) {
					activeRequest.act();
					newRequest = false;
				} 

				if(activeRequest == null) {
					if(queuedRequests.isEmpty()) {
						allRequestsCompleted = true;
					} else {
						setActiveRequest(queuedRequests.remove(0));
					}
				} else if(activeRequest.isFinished()) {
					activeRequest = null;
					System.out.println("Request finished");
				}
				
				
			}

		}

		@Override
		public void onStop(double timestamp) {
			
		}
		
	};
	
	public void enableCompressor(boolean enable){
		compressor.enableDigital();
	}

	@Override
	public void stop() {
	}

	@Override
	public void zeroSensors() {
		//turret.zeroSensors();
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}

	@Override
	public void outputTelemetry() {
	}

	public Request waitRequest(double seconds){
		return new Request(){
			double startTime = 0.0;
			double waitTime = 1.0;
		
			@Override
			public void act() {
				startTime = Timer.getFPGATimestamp();
				waitTime = seconds;
			}

			@Override
			public boolean isFinished(){
				return (Timer.getFPGATimestamp() - startTime) > waitTime;
			}
		};
	}

	public Request waitForVisionRequest(){
		return new Request(){

			@Override
			public void act() {

			}

			/*@Override
			public boolean isFinished(){
				return robotState.seesTarget();
			}*/

		};
	}


	private boolean needsToNotifyDrivers = false;
    public boolean needsToNotifyDrivers() {
        if (needsToNotifyDrivers) {
            needsToNotifyDrivers = false;
            return true;
        }
        return false;
	}


	public void shootingBottomState() {
		request(
			new ParallelRequest(
				actuatingHood.stateRequest(ActuatingHood.State.FAR),
				shooter.openLoopRequest(0.34),
				feeder.feederStageRequest()
			),
			new SequentialRequest(
				feeder.stateRequest(Feeder.State.FEEDINGSECOND),
				feeder.waitToFinishShootingRequest(),
				feeder.stateRequest(Feeder.State.FEEDINGSECOND),
				postShootingRequest()
			)
			//postShootingRequest()
		);
	}

	public void shootingTopState() {
		request(
			new ParallelRequest(
				turret.angleRequest(0.0),
				actuatingHood.stateRequest(ActuatingHood.State.NOVISION),
				shooter.holdWhenReadyRequestRequest(6000),
				feeder.feederStageRequest()
			),
			new SequentialRequest(
				waitRequest(0.25),
				feeder.stateRequest(Feeder.State.FEEDING),
				feeder.waitToFinishShootingRequest(),
				feeder.stateRequest(Feeder.State.FEEDINGSECOND),
				postShootingRequest()
			)
			//postShootingRequest()
		);
	}

	public void shootingState() {
		request(
			new ParallelRequest(
				//accelWheels.visionHoldWhenReadyRequest(), //4/3 is the ratio for surface speed for the flywheels
				shooter.visionHoldWhenReadyRequest(),
				feeder.feederStageRequest()
			),
			new SequentialRequest(
				feeder.stateRequest(Feeder.State.FEEDING),
				feeder.waitToFinishShootingRequest(),
				feeder.feederStageRequest(),
				shooter.waitToHoldRequest(),
				//accelWheels.waitToHoldRequest(),
				//waitRequest(0.125),
				feeder.stateRequest(Feeder.State.FEEDINGSECOND),
				feeder.waitToFinishShootingRequest(),
				postShootingRequest()
			)
			//postShootingRequest()
		);
	}

	public void autoShootingState() {
		request(
			new ParallelRequest(
				//accelWheels.visionHoldWhenReadyRequest(), //4/3 is the ratio for surface speed for the flywheels
				shooter.visionHoldWhenReadyRequest(),
				feeder.feederStageRequest()
			),
			new SequentialRequest(
				feeder.stateRequest(Feeder.State.FEEDINGSECOND),
				feeder.waitToFinishShootingRequest(),
				shooter.waitToHoldRequest(),
				waitRequest(0.5),
				feeder.stateRequest(Feeder.State.FEEDINGSECOND),
				feeder.waitToFinishShootingRequest(),
				postShootingRequest()
			)
			//postShootingRequest()
		);
	}

	public void setFeederStateStaging() {
		System.out.println("superstructure");
		request(
			feeder.feederStageRequest()
		);
	}

	public void testShootingState(double RPM){
		request(
		new SequentialRequest(shooter.holdWhenReadyRequestRequest(RPM))	
		);
	}

	public void ShootBalls() {
		request(
			new SequentialRequest(
				shooter.visionHoldWhenReadyRequest(),
				feeder.stateRequest(Feeder.State.FEEDING),
				feeder.waitToFinishShootingRequest(),
				feeder.stateRequest(Feeder.State.FEEDING),
				feeder.waitToFinishShootingRequest()
			), 
			postShootingRequest()
		);
	}




	

	public SequentialRequest postShootingRequest() {
		return new SequentialRequest (
			feeder.timedReverseRequest(0.25),
			new ParallelRequest(
				//new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(false)),
				shooter.openLoopRequest(0.0),
				feeder.stateRequest(Feeder.State.IDLE),
				intake.stateChangeRequest(Intake.State.IDLE),
			
				//actuatingHood.stateRequest(ActuatingHood.State.CLOSE),
				turret.stateRequest(Turret.State.POSITION)
			)
		);
	}
	public void candleState(Candle.State state, int r, int g, int b) {
		candle.stateChange(state, r, g, b);
	}
	public void candleStateRequest(Candle.State state, int r, int g, int b) {
		request(
			candle.stateChangeRequest(state, r, g, b)
		);
	}
	public void candleStateRequest(Candle.State state) {
		request(
			candle.stateChangeRequest(state)
		);
	}
	public void candleSetLEDs( int r, int g, int b,int w, int start, int length) {
		request(
			candle.setLEDs( r, g, b,w, start, length)
		);
	}
	public void candleDefaultLEDRequest() {
		request(
			candle.defualtLEDRequest()
		);
	}
	public void candleClimberLEDRequest() {
		request(
			candle.climberLEDRequest()
		);
	}

	/**
     * Creates a CanDLE Stripe
     * @param lengthOStipe how long the stripe is
	 * @param startOStripe where does it start
	 * @param speedOStripe how fast the stripe should move
	 * @param redOStripe R in the RGB component
	 * @param greenOStripe G in the RGB component
	 * @param blueOStripe B in the RGB component
	 * @param validAreaStartOStripe where can the stripe move around
	 * @param validAreaEndOStripe where the stripe can end.
     */
	public void createCandleStripe(int lengthOStipe, int startOStripe, Double speedOStripe, int redOStripe, int greenOStripe, int blueOStripe, int validAreaStartOStripe, int validAreaEndOStripe, String idOstripe) {
		request(
			candle.createStripeRequest(lengthOStipe, startOStripe, speedOStripe, redOStripe, greenOStripe, blueOStripe, validAreaStartOStripe, validAreaEndOStripe, idOstripe)
		);
	}
	public void deleteCandleStripe(String deletionId) {
		request(
			candle.deletionRequest(deletionId)
		);
	}
	
	public void candleSetIsIntaking(boolean isIntaking) {
		candle.setIntaking(isIntaking);
		/*request(
			candle.setIntaking(isIntaking)
		);*/
	}
	public void candleSetClimbing(boolean isClimbing) {
		candle.setClimberEnabled(isClimbing);
		/*request(
			new ParallelRequest(
				candle.setClimberEnabled(isClimbing)
				
			)
		);*/
	}

	public void intakeState() {
		request(
			new ParallelRequest(
				intake.stateChangeRequest(Intake.State.INTAKING),
				feeder.stateRequest(Feeder.State.INTAKING)	
			)
		);
	}


	public void feedstate() {
		request(
			new ParallelRequest(
				//intake.stateChangeRequest(Intake.State.INTAKING)
				feeder.stateRequest(Feeder.State.INTAKING)	
			)
		);
	}

	public void bruhWeClimbin(){
		request(
			new SequentialRequest(
				turret.angleRequest(0),
				climber.fireMiniExtenderRequest(false)
			)
	
		);
	}

	public void feedstopstate() {
		request(
			new ParallelRequest(
				//intake.stateChangeRequest(Intake.State.INTAKING)
				feeder.stateRequest(Feeder.State.IDLE)	
			)
		);
	}

	/*public void setRequest(double signal) {
		request(
			new ParallelRequest(
				actuatingHood.moveRequest(signal)
				
			)
		);
	} */

	public void intakeReverse() {
		request(
			new ParallelRequest(
				intake.stateChangeRequest(Intake.State.EJECTING),
				feeder.stateRequest(Feeder.State.REVERSE)
				
			)
		);
	}

	public void intakeStopState() {
		request(
			new ParallelRequest(
				intake.stateChangeRequest(Intake.State.IDLE),
				feeder.stateRequest(Feeder.State.IDLE)
			)
		);
	}

	public void turretMoveRequest(){
		request(
			new ParallelRequest(
				turret.angleRequest(-90, 0.3)
			)
		);

	}

	public void turretOpenLoop(double signal){
		request(
			new ParallelRequest(
				turret.openLoopRequest(signal)
			)
		);

	}
	public void turretTurn90(){
		request(
			new ParallelRequest(
				turret.angleRequest(-90,0.6)
			)
		);

	}
	public void turretTurn(double angle){
		request(
			new ParallelRequest(
				turret.angleRequest(angle,0.5)
			)
		);

	}

	public void neutralState(){
		clearQueue();
		System.out.println("neutral state running");
		request(
			new ParallelRequest(
				intake.stateChangeRequest(Intake.State.IDLE),
				feeder.stateRequest(Feeder.State.IDLE),
				shooter.openLoopRequest(0.0),
				turret.stateRequest(Turret.State.POSITION)
			)
		);

	}	

	public void servoFarRequest(){
		request(
			new ParallelRequest(
				actuatingHood.stateRequest(ActuatingHood.State.FAR)
			)
		);

	}

	public void servoCloseRequest(){
		request(
			new ParallelRequest(
				actuatingHood.stateRequest(ActuatingHood.State.CLOSE)
			)
		);

	}
	public boolean shooterAtRPM(){
		return shooter.hasReachedSetpoint();
	}

	public void firingVisionState() {
		request( 
			new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
			new SequentialRequest(
				new ParallelRequest(
					actuatingHood.visionExtensionRequest(),
					shooter.visionHoldWhenReadyRequest(), // 400.0 3000.0 | 1100 3600
					turret.startVisionRequest()
				),
				
				new ParallelRequest(
					intake.stateChangeRequest(Intake.State.FEEDING),
					feeder.stateRequest(Feeder.State.FEEDING),
					feeder.waitToFinishShootingRequest()
				),
				postShootingRequest()
			)
		);
	}

	public void visionLockOnRequest(){
		request(
			new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
			new ParallelRequest(
				turret.startVisionRequest(),
				actuatingHood.visionExtensionRequest()
				
				//candle.createStripeRequest(71, 0, 0.0, 255, 0, 255, 0, 70, "vision lock on")
			)
		);
	}

	public void turretVision(){
		request(
			new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
			new ParallelRequest(
				turret.startVisionRequest()
				
				//candle.createStripeRequest(71, 0, 0.0, 255, 0, 255, 0, 70, "vision lock on")
			)
		);
	}
	public void goodVisionLockOnRequest(){
		request(
			new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
			new ParallelRequest(
				//actuatingHood.visionExtensionRequest(),
				turret.startVisionRequest()
				//candle.createStripeRequest(71, 0, 0.0, 255, 0, 255, 0, 70, "vision lock on")
			)
		);
	}


	public void ClimberExtend(double inches){
		request(
			new SequentialRequest(
				turret.angleRequest(0, 0.6),
				climber.setHeightRequest(inches, 1) //inchos
			)

		);
	}


	public void ClimberRetract(){
		request(
			new SequentialRequest(
				climber.setHeightRequest(0, 0.5) 
			)

		);
	}

	public void LockClimber(){
		request(
			new SequentialRequest(
				climber.lockHeightRequest()
			)

		);
	}

	public void ClimberExtendPiston(boolean extend){
		request(
			new SequentialRequest(
				climber.pistonMove(extend)
			)

		);
	}
	public void fireMiniExtender(boolean extend){
		request(
			new SequentialRequest(
				climber.fireMiniExtenderRequest(extend)
			)

		);
	}


	/////States/////



}
