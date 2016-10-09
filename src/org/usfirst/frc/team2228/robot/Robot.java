
package org.usfirst.frc.team2228.robot;


import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * 
 * Class: Robot
 * Author: HARRY 
 * Date: 2/27/2016
 * 
 */
public class Robot extends IterativeRobot {

	
	//Object Variables
	private Joystick joy;
	private Shooter shooter;
	private CANTalon leftMaster;
	private CANTalon rightMaster;
	private CANTalon leftSlave;
    private CANTalon rightSlave;
    private Aquirer aquire;
    private BuiltInAccelerometer accel;
    private CameraServer server;
    private Debug debug;
    
 // Auto Defense choices
    enum Defense { LOW_BAR, CHEVAL_de_FRISE, GENERAL }
    Defense defense;
    
    //Autonomous States
    enum State { UNKNOWN,
    	         APPROACH_LOW_AQUIRE, CROSSING_LOW_AQUIRE,
    	         APPROACH_CHEVAL, CROSS_CHEVAL, CHEVAL_ARMS_READY, CHEVAL_MOVE_FWD,
    	         APPROACH_GENERAL,CROSS_GENERAL, 
    	         NOTHING,
    	         TELEOP }
    State state;
    
    //Teleop drive variables
    private double rSpeed = 0;
    private double lSpeed = 0;
    private double speedDivisor = 1;
    
    


    /**
     * robotInit called upon start up, instantiates all objects and primitives
     * as well as setting defaults
     * 
     */
    public void robotInit() {
    	
        server = CameraServer.getInstance();
        server.setQuality(50);
        
        //the camera name (ex "cam0") can be found through the roborio web interface
        server.startAutomaticCapture("cam1");
    	
    	accel = new BuiltInAccelerometer();
    	joy = new Joystick(RobotMap.JOYSTICK_PORT);
    
    	SmartDashboard.putNumber("offsetx", 0);
    	SmartDashboard.putNumber("offsety",0);
    	SmartDashboard.putBoolean("found", false);
    	SmartDashboard.putBoolean("vision", false);
    	
    	rightMaster = new CANTalon(RobotMap.RIGHT_MOTOR_MSTR_PORT);
    	rightSlave = new CANTalon(RobotMap.RIGHT_MOTOR_SLV_PORT);
    	leftMaster = new CANTalon(RobotMap.LEFT_MOTOR_MSTR_PORT);
    	leftSlave= new CANTalon(RobotMap.LEFT_MOTOR_SLV_PORT);
    	    	
    	shooter = new Shooter(RobotMap.SHOOTER_ANGLE_PORT,RobotMap.SHOOTER_WHEEL_LEFT_PORT,RobotMap.SHOOTER_WHEEL_RIGHT_PORT,RobotMap.BALL_DETECT_PORT,RobotMap.SHOOTER_SERVO_PORT);
    	aquire = new Aquirer(RobotMap.AQUIRE_ARM_ANGLE_PORT,RobotMap.GATHER_TALON_PORT);
    	    	
    	SmartDashboard.putNumber("right1MotorVoltage", 0);
    	SmartDashboard.putNumber("rightSlaveMotorVoltage", 0);
    	SmartDashboard.putNumber("left1MotorVoltage", 0);
    	SmartDashboard.putNumber("leftSlaveMotorVoltage", 0);

    	
	  	rightMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
	    rightMaster.reverseSensor(false);

	      rightMaster.configNominalOutputVoltage(+0.0f, -0.0f);
	      rightMaster.configPeakOutputVoltage(+1.0f, -1.0f);
	      rightMaster.setProfile(0);
	      rightMaster.setF(0);
	      rightMaster.setP(0.21);
	      rightMaster.setI(0.0001); 
	      rightMaster.setD(0);
	      
	      

      
      
    	  leftMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
          leftMaster.reverseSensor(false);
          leftMaster.configNominalOutputVoltage(+0.0f, -0.0f);
          leftMaster.configPeakOutputVoltage(+1.0f, -1.0f);
          leftMaster.setProfile(0);
          leftMaster.setF(0);
          leftMaster.setP(0.21);
          leftMaster.setI(0.0001); 
          leftMaster.setD(0);
         
          defense = Defense.CHEVAL_de_FRISE;
          state = State.UNKNOWN; 
          SmartDashboard.putNumber("angleS", 397.2);
          SmartDashboard.putNumber("Defense", 1);
         
    }
    
  
    /**
     * 
     */
    public void autonomousInit() {


	        rightMaster.setPosition(0);
	    	leftMaster.setPosition(0);
	    	shooter.setFirst(false);
	    	aquire.setFirst(false);
	    	if(SmartDashboard.getNumber("Defense") == 1){
	    		//for lowbar and portcullis
	    		state = State.APPROACH_LOW_AQUIRE;
	    	}else if(SmartDashboard.getNumber("Defense") == 2){
	    		//for cheval
	    		state = State.APPROACH_CHEVAL;
	    	}else if(SmartDashboard.getNumber("Defense") == 3){
	    		//rock wall, moat, ramparts
	    		state = State.APPROACH_GENERAL;
	    	}else{
	    		//staying still and calibrating
	    		state = State.NOTHING;
	    	}

    }
    
    /**
     * 
     */
    public void resetInTeleop() {


	        rightMaster.setPosition(0);
	    	leftMaster.setPosition(0);
	    	shooter.setFirst(false);
	    	aquire.setFirst(false);
	    	aquire.setHere(false);
	    	aquire.setHere4(false);

	    	
	    	System.out.println("AutoInitRun");
	    	aquire.setOnce(false);

    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {


    	switch (state){
    	
          // make sure auto init was called ?
    	  case UNKNOWN: {
    		  autonomousInit();
    		  break;
    	  }

    	  
    	  /************************ low bar *************************/
    	  case APPROACH_LOW_AQUIRE: {
    	   		
  	    	if(!shooter.calibrated()){
  	    		shooter.calibrate();
  	    	}
  	    	
  	    	if(!aquire.calibrated()){
  	    		aquire.calibrate();
  	    	}
  	    	
  	    	//2.583 = wheel revs to go 58 inches
  	    	rightMaster.set(-RobotMap.COUNTS_PER_REV_DRIVE *RobotMap.WHEEL_REVS_TO_DEFENSE);
      		leftMaster.set(RobotMap.COUNTS_PER_REV_DRIVE *RobotMap.WHEEL_REVS_TO_DEFENSE);   		  

    	    
	    	if(rightMaster.getPosition() < RobotMap.COUNTS_PER_REV_DRIVE*RobotMap.WHEEL_REVS_TO_DEFENSE+50){
	    		rightMaster.setPosition(0);
	    		leftMaster.setPosition(0);
	    		rightMaster.set(0);
	    		leftMaster.set(0);
	    		state = State.CROSSING_LOW_AQUIRE;
	        }
    	    if(rightMaster.getOutputCurrent()  > 50){
    		
    	    }
	        break;
    	  }
    	  
    	  case CROSSING_LOW_AQUIRE: {
    	    		if(shooter.checkLimitStatusForward() && aquire.checkLimitStatusForward()){

    	    			//2.489 = wheel revs per 52
    	    			rightMaster.set(-RobotMap.COUNTS_PER_REV_DRIVE*RobotMap.WHEEL_REVS_ACROSS_DEFENSE);
    	    			leftMaster.set(RobotMap.COUNTS_PER_REV_DRIVE*RobotMap.WHEEL_REVS_ACROSS_DEFENSE);
    	    		}else{
    		    		aquire.calibrate();
    		    		shooter.calibrate();
    	    		}
    		  break;
    	  }

    	  /************************ low bar *************************/    	
  
    	  
    	  /************************ cheval de frise *************************/     	  
    	case APPROACH_CHEVAL: {
	    	if(!shooter.calibrated()){
	    		shooter.calibrateUp();
	    		System.out.println("RUNNING THIS");
	    	}else{
	    			shooter.fightdaPOWA();
	    		
	    	}
	    	
	    	if(!aquire.calibrated()){
	    		aquire.calibrateUp();
	    	}else{
	        	aquire.overCheval();
	    	}
	    	rightMaster.set(-RobotMap.COUNTS_PER_REV_DRIVE*RobotMap.WHEEL_REVS_TO_DEFENSE+600);
    		leftMaster.set(RobotMap.COUNTS_PER_REV_DRIVE*RobotMap.WHEEL_REVS_TO_DEFENSE-600);
	    		if(rightMaster.getPosition() <-RobotMap.COUNTS_PER_REV_DRIVE*RobotMap.WHEEL_REVS_TO_DEFENSE+1200){
		    		rightMaster.setPosition(0);
		    		leftMaster.setPosition(0);
		    		rightMaster.set(0);
		    		leftMaster.set(0);
		    		state = State.CROSS_CHEVAL;
		    		
		    	}
	    		break;
    	}
    	case CROSS_CHEVAL: {
    		
    		if(!aquire.getHere()){
    			aquire.fightdaPOWA();  // raise over 5900
    			shooter.fightdaPOWA();
    		}else{
    			state = State.CHEVAL_ARMS_READY;   // CHEVAL_ARMS_READY
    		}
    		break;
    	}
    	
    	case CHEVAL_ARMS_READY: {

			rightMaster.set(-RobotMap.COUNTS_PER_REV_DRIVE*0.527-900);
			leftMaster.set(RobotMap.COUNTS_PER_REV_DRIVE*0.527+900);
			
    		if(rightMaster.getPosition()<-RobotMap.COUNTS_PER_REV_DRIVE*0.527-600){
    			state = State.CHEVAL_MOVE_FWD;
    			aquire.overCheval(); // to reverse switch or 1700
    			rightMaster.setPosition(0);
	    		leftMaster.setPosition(0);
	    		rightMaster.set(0);
	    		leftMaster.set(0);
    		}			
			break;
    	}
    	
    	case CHEVAL_MOVE_FWD: {
    			aquire.overCheval();

    			rightMaster.set(-RobotMap.COUNTS_PER_REV_DRIVE*0.527-1300);
    			leftMaster.set(RobotMap.COUNTS_PER_REV_DRIVE*0.527+1300);
    			break;
    		}
    	
       /*************************** end cheval de frise *******************/
    	
    	case APPROACH_GENERAL: {
	    	if(!shooter.calibrated()){
	    		shooter.calibrateUp();
	    		System.out.println("RUNNING THIS");
	    	}else{
	    			shooter.fightdaPOWA();
	    	}
	    	
	    	if(!aquire.calibrated()){
	    		aquire.calibrateUp();
	    	}else{
	        		aquire.overWall();
	    	}
	    	rightMaster.set(-RobotMap.COUNTS_PER_REV_DRIVE*RobotMap.WHEEL_REVS_TO_DEFENSE-200);
    		leftMaster.set(RobotMap.COUNTS_PER_REV_DRIVE*RobotMap.WHEEL_REVS_TO_DEFENSE);
	    		if(rightMaster.getPosition() <(-RobotMap.COUNTS_PER_REV_DRIVE*RobotMap.WHEEL_REVS_TO_DEFENSE-200)+50){
		    		rightMaster.setPosition(0);
		    		leftMaster.setPosition(0);
		    		rightMaster.set(0);
		    		leftMaster.set(0);
		    		state = State.CROSS_GENERAL;
		    	}
	    	break;
    	}
    	
    	case CROSS_GENERAL: {
    	
    		shooter.fightdaPOWA();
    		aquire.overWall();
    		
    		if(rightMaster.getPosition() <((-RobotMap.COUNTS_PER_REV_DRIVE*RobotMap.WHEEL_REVS_TO_DEFENSE/2))+50){
	    		rightMaster.setPosition(0);
	    		leftMaster.setPosition(0);
	    		rightMaster.set(0);
	    		leftMaster.set(0);
    		}else{
    			rightMaster.set((RobotMap.COUNTS_PER_REV_DRIVE*RobotMap.WHEEL_REVS_TO_DEFENSE)/2);
    			leftMaster.set((RobotMap.COUNTS_PER_REV_DRIVE*RobotMap.WHEEL_REVS_TO_DEFENSE)/2);
    		}
  
    	}
    		
    	case NOTHING: {
    		rightMaster.setPosition(0);
    		leftMaster.setPosition(0);
    		rightMaster.set(0);
    		leftMaster.set(0);
    		aquire.calibrate();
    		shooter.calibrate();
    		break;
    	}
    	}
    }


    
    /**
     * This function is called periodically during operator control
     */
	public void teleopPeriodic() {

    	if(state != State.TELEOP){
	      rightSlave.changeControlMode(TalonControlMode.Follower);
	      rightSlave.set(rightMaster.getDeviceID());
	      leftSlave.changeControlMode(TalonControlMode.Follower);
          leftSlave.set(leftMaster.getDeviceID());
          state = State.TELEOP;
    	}
     	rSpeed = -joy.getMagnitude();
    	lSpeed = -joy.getMagnitude();
    	
		
    	if(joy.getRawButton(12)){

    		resetInTeleop();
    		
    	}

    	
    	if(joy.getRawButton(1)){
			shooter.setMode("shootOptimal");
		}else{
			shooter.setMode("not");
		}
    	
    	if(joy.getRawButton(2)){
			shooter.setMode("gather");
		}
    	
        if(joy.getRawButton(3)){
        	shooter.setMode("aimhigh");
        }
        
		if(joy.getRawButton(4)){
			shooter.setMode("reset");
			shooter.setShooting(false);
		}
		
		if(joy.getRawButton(5)){
			shooter.setMode("calibratedown");
			shooter.setShooting(false);

		}
		
		if(joy.getRawButton(6)){
			shooter.setMode("bring_aquire_arm_down");
		}
		
		if(joy.getRawButton(7)){
			shooter.setMode("lowgoal");
		}
		
		if(joy.getRawButton(8)){
			shooter.setMode("port");
		}
		if(joy.getRawButton(9)){
			shooter.setMode("maybe");
		}
		if(joy.getRawButton(10)){
			shooter.setMode("testUp");
		}
	
		
		
		if(shooter.getMode().equals("startup")){
			shooter.calibrate();
			aquire.calibrate();
		}else if(shooter.getMode().equals("gather")){
			shooter.gather();
			aquire.gather();
		}else if(shooter.getMode().equals("shootOptimal")){
			
			shooter.startShoot();
			aquire.reset();
		}else if(shooter.getMode().equals("aimhigh")){
			shooter.aimUp2();
			aquire.reset();
		}else if(shooter.getMode().equals("calibratedown")){
			shooter.calibrate();
			aquire.calibrate();
		}else if(shooter.getMode().equals("bring_aquire_arm_down")){
			shooter.fightdaPOWA();
			aquire.fightdaPOWA();
		}else if(shooter.getMode().equals("lowgoal")){
			
			shooter.aimLow();
			aquire.overCheval();
			
		}else{
			
			if(!shooter.getShooting()){
				shooter.fightdaPOWA();
			}else{
				shooter.aimUp2();
			}

			aquire.overCheval();
			
		}
		
		
	
		
		if(joy.getAxis(Joystick.AxisType.kY) > 0.15){
    		
    		lSpeed *= 1;
    		rSpeed *= -1;
    		
    	}else if(joy.getAxis(Joystick.AxisType.kY) < -0.15){
    		
    		lSpeed *= -1;
    		rSpeed *= 1;
    		
    	}
    	
    	if(joy.getTwist()>0.15){
    		lSpeed+= joy.getTwist();
    		rSpeed+= joy.getTwist();

    	}else if(joy.getTwist() < -0.15){
    		lSpeed+= joy.getTwist();
    		rSpeed+= joy.getTwist();

    	}
    		
    	
    	if(joy.getRawButton(11)){
    			SmartDashboard.putBoolean("vision", true);
    	}else{
    			SmartDashboard.putBoolean("vision", false);
    	}
    	
    	
    	if(!joy.getRawButton(11)){
    		leftMaster.changeControlMode(TalonControlMode.PercentVbus);
    		rightMaster.changeControlMode(TalonControlMode.PercentVbus);

	    	leftMaster.set(lSpeed/speedDivisor);
	    	rightMaster.set(rSpeed/speedDivisor);
	    	rightMaster.setPosition(0);
	    	leftMaster.setPosition(0);
    	}
    	else if(SmartDashboard.getBoolean("found")){
    		if(SmartDashboard.getNumber("offsetX") >15){
    			rSpeed = -(SmartDashboard.getNumber("offsetX")/200) - 0.3;
    			lSpeed = -(SmartDashboard.getNumber("offsetX")/200) - 0.3;
    		}else if(SmartDashboard.getNumber("offsetX") <-15){
    			rSpeed = -(SmartDashboard.getNumber("offsetX")/200) + 0.3;
    			lSpeed = -(SmartDashboard.getNumber("offsetX")/200) + 0.3;
    		}else{
    			rSpeed = 0;
    			lSpeed = 0;
    		}
    		leftMaster.set(lSpeed/speedDivisor);
	    	rightMaster.set(rSpeed/speedDivisor);
    	}
    	
    	SmartDashboard.putNumber("right1MotorVoltage", rightMaster.getOutputCurrent());
    	SmartDashboard.putNumber("right2MotorVoltage", rightSlave.getOutputCurrent());
    	SmartDashboard.putNumber("left1MotorVoltage", leftMaster.getOutputCurrent());
    	SmartDashboard.putNumber("left2MotorVoltage", leftSlave.getOutputCurrent());

    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    	accel.startLiveWindowMode();

    }
    
}
