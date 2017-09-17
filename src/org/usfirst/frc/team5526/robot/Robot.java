package org.usfirst.frc.team5526.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
By: Bernardo Oviedo

Code for the 2017 First Robotics Competition.
The program is open source and free to use.

NOTE: it is recommended not to directly copy the code but rather take the core concepts
from different parts of the code that can later be implemented in your own robot. 
*/

public class Robot extends SampleRobot {
	
	//----------------------------------OBJECTS---------------------
	RobotDrive myRobot; // class that handles basic drive
											
	//Camera Object
	CameraServer server;
	
	//Sensor Objects
	ADXRS450_Gyro gyro;
	
	//Joystick Object
	Joystick leftStick; // set to ID 1 in DriverStation
	Joystick rightStick;
	
	//Pneumatic Objects
	Solenoid solenoid1, solenoid2, pistonSides1, pistonSides2;	
	
	//Motor Objects
	Victor lanzadorIzquierdo, lanzadorDerecho, levantador, alimentador, escalador, escaladorIzquierdo;
	
	//Encoder Objects
	Encoder encoder;
	
	//Dashboard Object
	DriverStation ds;
	
	//-------------------------------PORTS----------------------
	//Button Ports
	static final int rotateLeft = 5;
	static final int rotateRight = 3;
	
	//Motor Ports
	static final int leftMotor = 0;
	static final int rightMotor = 1;
	static final int escaladorPortIzquierdo = 3;
	static final int lanzadorDerechoPort = 4;
	//static final int levantadorPort = 3;
	static final int alimentadorPort = 6;
	static final int escaladorPort = 2;
	
	//Pneumatic Ports
	static final int solenoidPort1 = 2;
	static final int solenoidPort2 = 3;
	static final int pistonSidesPort1 = 0;
	static final int pistonSidesPort2 = 1;
	
	//Robot Expiration && Update
	static final double robotExpiration = 0.1;
	private final double kUpdatePeriod = 0.005;
	
	//Chooser
	SendableChooser<String> chooser;
	
	double angle;
	
	//---------------------------------------PIDs---------------------------------------
	
	//Drive Straight PID variables	
	static final double Kp = 0.72;
	
	//------------------------------Initialize Ports-----------------------------
	public Robot(){
		server = CameraServer.getInstance();
		server.startAutomaticCapture();
		myRobot = new RobotDrive(leftMotor,rightMotor);
		myRobot.setMaxOutput(0.55);
		leftStick = new Joystick(1);
		rightStick = new Joystick(0);
		myRobot.setExpiration(0.1);
		solenoid1 = new Solenoid(solenoidPort1);
		solenoid2 = new Solenoid(solenoidPort2);
		pistonSides1 = new Solenoid(pistonSidesPort1);
		pistonSides2 = new Solenoid(pistonSidesPort2);
		gyro = new ADXRS450_Gyro();
		escaladorIzquierdo = new Victor(escaladorPortIzquierdo);
		lanzadorDerecho = new Victor(lanzadorDerechoPort);
		//levantador = new Victor(levantadorPort);
		alimentador = new Victor(alimentadorPort);
		escalador = new Victor(escaladorPort); 
		encoder = new Encoder(0, 1, true, EncodingType.k4X);
		ds = DriverStation.getInstance();
		encoder.setDistancePerPulse(0.001956947162); //529
		gyro.calibrate();
		chooser = new SendableChooser<>();
		chooser.addDefault("Default Auto", "Default");
		chooser.addObject("My Auto", "My Auto");
		SmartDashboard.putData("Auto modes", chooser);
	}
	
	//-------------------------------USER DEFINED METHODS-------------------------------
		public void rotate(int angle, double velocity){
			gyro.reset();
			Timer.delay(0.005);
			
			if(angle > 0){		
				while(gyro.getAngle() < angle){ // 31 for MAX speed
					myRobot.arcadeDrive(0.0, -velocity);
					Timer.delay(0.005);
				}
			}
			if(angle < 0){
				while(gyro.getAngle() > angle){ // 31 for MAX speed
					//System.out.println("Gyro Angle: "+gyro.getAngle());
					myRobot.arcadeDrive(0.0, velocity);
					Timer.delay(0.005);
				}
			}
			
			myRobot.stopMotor();
		}
		
		public void driveStraight(double distance, boolean direction, double vel, double Kp){
			double velocity = 0;
			
			if(direction){
				velocity = -vel;
			}
			else{
				velocity = vel;				
			}
			
			gyro.reset();
			encoder.reset();
			Timer.delay(0.005);
			double currentDistance = encoder.getDistance();
			
			while(currentDistance < distance){
				double angle = gyro.getAngle();
				myRobot.arcadeDrive(velocity, angle*Kp);
				currentDistance = encoder.getDistance();
				
				if(!direction){
					currentDistance = -currentDistance;
				}
				
				Timer.delay(0.005);
			}
			myRobot.stopMotor();
		}
		
	//----------------------------------AUTONOMOUS-------------------------------
	public void autonomous(){	
		myRobot.stopMotor();
		encoder.reset();
		gyro.reset();
		myRobot.setMaxOutput(0.55);

		boolean move = true;
		boolean bonus = true;
		boolean enter = true;
		int angleRotate = 0;
		
		boolean posicionIzquierda = false;
		boolean posicionMedia = false;
		boolean posicionDerecha = false;
		boolean colorBool = true;
		
		//DriverStation.Alliance color;
		
		//color = DriverStation.getInstance().getAlliance();		
		//colorBool = SmartDashboard.getBoolean("New Name", false);
		posicionIzquierda = SmartDashboard.getBoolean("DB/Button 1", false);
		posicionMedia = SmartDashboard.getBoolean("DB/Button 2", false);
		posicionDerecha = SmartDashboard.getBoolean("DB/Button 3", false);
		
		// color = DriverStation.Alliance.Red
		
		//Dashboard if button0/New Name is pressed Red Alliance will be activated
		
				
		while(isAutonomous() && isEnabled()){
			
			//-----------------------RED------------------
			if(colorBool){
				 
				//Posicion Izquierda				
				if(posicionIzquierda){
					//System.out.println("Posicion Izquierda");
					//0.74 m largo del robot Prototipo
					if(move){
						//driveStraight(3.5, true, 0.75, 0.72); //2.20 --> 2.26
						driveStraight(2.09, true, 0.72, 0.72);  //Distance: 2.15
						Timer.delay(0.1);
						rotate(60, 1.0);
						Timer.delay(0.1);
						driveStraight(0.54, true, 0.83, 0.35); //0.85 Distance: 0.72
						/*Timer.delay(0.08);
						pistonSides1.set(false);
						pistonSides2.set(true);
						Timer.delay(1);
						solenoid1.set(false);
						solenoid2.set(true);
						Timer.delay(1);
						pistonSides1.set(true);
						pistonSides2.set(false);
						//Timer.delay(1);
						solenoid1.set(true);
						solenoid2.set(false);
						Timer.delay(0.05);
						driveStraight(0.2, false, 1.0, 0.35);
						Timer.delay(0.1);*/
						/*rotate(-40, 1.0); //60
						driveStraight(0.5, true, 1.0, 0.0);*/
						move = false;
					}
				}
				
				//Posicion Media
				if(posicionMedia){
					angleRotate = -45;
					myRobot.stopMotor();
					
					if(move){
						driveStraight(1.8, true, 0.70, Kp); // Distance: 1.8 Velocity: 0.70 | Distance: 1.34
						move = false;                        // Distance: 1.29 Velocity: 0.85
						myRobot.stopMotor();
					}
					
					if(enter){
						Timer.delay(0.5);
						pistonSides1.set(false);
						pistonSides2.set(true);						
						Timer.delay(1); 
						solenoid1.set(false);
						solenoid2.set(true);
						Timer.delay(0.5);
						solenoid1.set(true);
						solenoid2.set(false);
						pistonSides1.set(true);
						pistonSides2.set(false);
						Timer.delay(1.0);
						encoder.reset();
						gyro.reset();
						driveStraight(0.3, false, 1.0, 0.0);
						myRobot.stopMotor();
						enter = false;
						Timer.delay(0.005);
					}
					
					/*if(bonus){
						rotate(angleRotate, 0.85);
						driveStraight(2.0, true, 0.9, 0.0);
						rotate(30, 0.85);
						driveStraight(1.0, true, 0.9, 0.0);
						bonus = false;
					}*/
					
				}	
				
				//Posicion Derecha
				if(posicionDerecha){
					//System.out.println("Posicion Derecha");
					if(move){
						//driveStraight(3.5, true, 0.75, 0.72); //2.20 --> 2.26
						driveStraight(2.1, true, 0.75, 0.72); 
						Timer.delay(0.1);
						rotate(-60, 0.91);
						Timer.delay(0.1);
						driveStraight(0.7, true, 0.83, 0.35); //0.85
						Timer.delay(0.08);
						pistonSides1.set(false);
						pistonSides2.set(true);
						Timer.delay(1);
						solenoid1.set(false);
						solenoid2.set(true);
						Timer.delay(1);
						pistonSides1.set(true);
						pistonSides2.set(false);
						//Timer.delay(1);
						solenoid1.set(true);
						solenoid2.set(false);
						Timer.delay(0.05);
						driveStraight(0.2, false, 1.0, 0.35);
						Timer.delay(0.1);
						/*rotate(-40, 1.0); //60
						driveStraight(0.5, true, 1.0, 0.0);*/
						move = false;					
					}

				}
				
			}
		}
	}
	
	
	@Override
	//----------------------------------TELEOP--------------------------------
	public void operatorControl() {
		myRobot.setSafetyEnabled(true);
		encoder.reset();
		myRobot.stopMotor();
		boolean buttonValue01;
		String autoSelected;
		
		//---------------------Initialize local variables-------------------
		double angle = 0;
		long contInterno = 0, contExterno = 0;
		double lanzadorThrottle = 0.0;
		double currentDistance = 0;	
		boolean meter = false, rotate = false, rotateLeftBool = false, rotateRightBool = false;
		boolean solenoidBool1, solenoidBool2, solenoids, straight, alimentadorBool, levantadorBool, pistonSidesBool1, pistonSidesBool2;
		straight = false; solenoidBool1 = false; solenoidBool2 = false; solenoids = false; alimentadorBool = false;
		levantadorBool = false; pistonSidesBool1 = false; pistonSidesBool2 = false;	
		myRobot.setMaxOutput(0.9);
		
		while(isOperatorControl() && isEnabled()) {
			//--------------------Control Arcade Drive------------------------------
			myRobot.arcadeDrive(leftStick.getRawAxis(1), -leftStick.getRawAxis(0), true);
			
			/*autoSelected = chooser.getSelected();
			System.out.println("Auto Selected: "+ autoSelected);
			buttonValue01 = SmartDashboard.getBoolean("DB/Button 1", false);
			if(buttonValue01) System.out.println("Button1 has been Pressed");*/
			
			//System.out.println("Angle: "+gyro.getAngle()+" Distance: "+encoder.getDistance()+" Direction: "+encoder.getDirection());
			
			//Buttons
			if(leftStick.getRawButton(3)) rotateLeftBool = true;
			if(leftStick.getRawButton(5)) rotateRightBool = true;
			if(leftStick.getRawButton(4)) solenoidBool1 = true;
			if(leftStick.getRawButton(6)) solenoidBool2 = true;
			if(leftStick.getRawButton(3)) solenoids = true;
			if(leftStick.getRawButton(2)) straight = true;
			if(rightStick.getRawButton(6)) meter = true;  
			if(rightStick.getRawButton(7)) alimentadorBool = true;
			if(rightStick.getRawButton(8)) alimentadorBool = false;
			//if(rightStick.getTrigger()) levantadorBool = true;
			if(rightStick.getRawButton(2)) levantadorBool = false;
			if(leftStick.getRawButton(7)) pistonSidesBool1 = true;
			if(leftStick.getRawButton(8)) pistonSidesBool2 = true;			
			
			//---------------------------ROTATIONS---------------------
			//Rotate Right
			if(leftStick.getRawButton(rotateRight)){
				myRobot.arcadeDrive(0.0, -1.0);
			}
			
			//Rotate Left
			if(leftStick.getRawButton(rotateLeft)){
				myRobot.arcadeDrive(0.0, 1.0);
			}
			
			//---------------------------Straight Driving---------------------
			if(straight){
				double Kp = 0.03;
				if(contInterno != contExterno || (contInterno == 0 && contExterno == 0) ){
					gyro.reset();
					contInterno = 0;
					contExterno = 0;
				}
				contInterno++;
				angle = gyro.getAngle();
				myRobot.arcadeDrive(-1.0, -angle*0.025, true);
				Timer.delay(0.01);
			}
			
			//---------------------------1 Meter Move-----------------------_
			if(meter){
				currentDistance = encoder.getDistance();
				gyro.reset();
				while(encoder.getDistance() < 0.8+currentDistance){
					angle = gyro.getAngle();
					myRobot.arcadeDrive(-0.7, angle*Kp, true);
					Timer.delay(0.01);
				}
				meter = false;
				
			}
			
			if(alimentadorBool){
				alimentador.set(0.3);
				//System.out.println("Alimentador Activado");			
			}
			else {
				alimentador.stopMotor();
				//System.out.println("Alimentador Desactivado");
			}
			
			//Process Button Bool Values
			if(solenoidBool1){
				solenoid1.set(true);
				solenoid2.set(false);
				solenoidBool1=false;
				solenoidBool2=false;
				solenoids = false;					
			}
			
			if(solenoidBool2){
				pistonSides1.set(false);
				pistonSides2.set(true);
				Timer.delay(0.9);
				solenoid1.set(false);
				solenoid2.set(true);
				solenoidBool2=false;
				solenoidBool1=false;
				solenoids = false;
				Timer.delay(0.15);
				pistonSides1.set(true);
				pistonSides2.set(false);
				Timer.delay(0.05);
				solenoid1.set(true);
				solenoid2.set(false);			
			}
			
			if(solenoids){
				solenoid1.set(false);
				solenoid2.set(false);
				solenoids = false;
				solenoidBool2=false;
				solenoidBool1=false;
			}
			
			if(pistonSidesBool1){
				pistonSides1.set(true);
				pistonSides2.set(false);
				pistonSidesBool1=false;
				pistonSidesBool2=false;					
			}
			
			if(pistonSidesBool2){
				pistonSides1.set(false);
				pistonSides2.set(true);
				pistonSidesBool2=false;
				pistonSidesBool1=false;
			}
			
			//-----------------------Lanzador-----------------------
			if(levantadorBool) {
				//levantador.set(-0.2);
			}
			else{
				//levantador.set(0.0);
			}
			
			//-----------------------Escalador-----------------------
			if(rightStick.getRawButton(5)) {
				escalador.set(-1.0);
				//System.out.println("Escalador Activado");
				escaladorIzquierdo.set(1.0);
			}
			
			/*if(rightStick.getRawButton(3)) {
				//escalador.set(-1.0);
				escaladorIzquierdo.set(1.0);
			}*/
			
			if(rightStick.getRawButton(3)) {
				escalador.set(0.0);
				//System.out.println("Escalador Desactivado");
				escaladorIzquierdo.set(0.0);
			}
			
			/*
			lanzadorIzquierdo.set(leftStick.getThrottle());
			lanzadorDerecho.set(leftStick.getThrottle());
			*/
			
			//Add SmartDashboard Values
			SmartDashboard.putNumber("Angle", gyro.getAngle());
			SmartDashboard.putNumber("Interno", contInterno);
			SmartDashboard.putNumber("Externo", contExterno);
			//SmartDashboard.putNumber("Left Sensor", leftSensorInt);
			SmartDashboard.putNumber("Throttle", lanzadorThrottle);
			
			straight = false;
			contExterno++;
			Timer.delay(kUpdatePeriod); // wait for a motor update time		
		}
		
		//solenoid1.set(false);
		//solenoid2.set(false);
	}
	
	public void test(){
		
	}
}
