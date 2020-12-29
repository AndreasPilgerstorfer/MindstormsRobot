//------------------------------------------------------------------------------------------
// RemoteTest.java
// Lego Mindstorms EV3 Project
//
// Authors:   Andreas Pilgerstorfer
//            Alexander Schönmann           
//
// Created:   10.11.2020
// Last changed: 19.12.2020
//
// This file contains a test class (RemoteTest), which allows the robot to run the specified
// code from the class InfraredSignalCheckerThread.java. This file also contains setting for
// the different sensors and motors.
//
// sources: 
//  http://www.rapidpm.org/2014/02/lego-mindstorms-ev3-components-infrared.html
//------------------------------------------------------------------------------------------
package remoteControl;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.NXTTouchSensor;
import lejos.robotics.RegulatedMotor;


public class RemoteTest {


    public static void main(String[] args) throws InterruptedException {
      
        // generating objects from the available sensors 
        final NXTTouchSensor rightTouch = new NXTTouchSensor(SensorPort.S1);
        final NXTTouchSensor leftTouch = new NXTTouchSensor(SensorPort.S2);
        final EV3IRSensor infraredSensor = new EV3IRSensor(SensorPort.S4);
        
        
        // generating objects from the available motors
        RegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
        RegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.B);
        RegulatedMotor armMotor = new EV3LargeRegulatedMotor(MotorPort.C);
        
        
        leftMotor.synchronizeWith(new RegulatedMotor[] {rightMotor});
        final InfraredSignalCheckerThread checkerThread = new InfraredSignalCheckerThread(infraredSensor, leftTouch, rightTouch, leftMotor,rightMotor, armMotor);
        
        
        // Settings
        checkerThread.isRunning = true;
        checkerThread.start();
        Button.waitForAnyPress();
        
        
        checkerThread.stop();
        checkerThread.isRunning = false;
        
        //Closing Sensors and Motors
        leftMotor.close();
        rightMotor.close();
        armMotor.close();
        
        rightTouch.close();
        leftTouch.close();
        infraredSensor.close();
       
        
    }
};