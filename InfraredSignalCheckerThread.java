//------------------------------------------------------------------------------------------
// InfraredSignalCheckerThread.java
// Lego Mindstorms EV3 Project
//
// Author:    Andreas Pilgerstorfer  
//            Alexander Shönmann
//
// Created:   10.11.2020
// Last changed: 19.12.2020
//
// This file contains a class (InfraredSignalCheckerThread), which is responsible to react on
// various inputs which are delivered by using an infrared remote control.
//
// Following methods are available:
//
// run()     reads the input from the remote control and allows the robot to drive in
//           different directions and starts the self-driving mode depending on the
//           provided input.
//
// drive()   Allows the robot to drive by himself using autopilot-mode. The robot detects 
//           table edges by himself due to the mounted touch sensors on both sides. He is 
//           also able to dodge barriers by using the infrared sensors. While using his third
//           motor the robot is capable of picking up Objects and dropping them at the 
//           desired location.
//
// customDrivingPace(int motorSpeed, int motorAccelaration)
//           setting driving speed and accelaration 
//
// customArmPace (int armSpeed, int armAccelaration)
//            setting arm speed and accelaration
//
// driveForwardAfterTurn () the robot drives forward until he reaches the table edge for the
//                          second time, drops the lego brick and drives backward 
//                   
// fetchLeft ()  gets the signals from the left touch sensor
//
// fetchRight ()  gets the signals from the right touch sensor
//
// obstacle () checks for Barriers in front of the robot.  If the value is smaller than a
//             specified amount, the robot returns true if not false. The first measurement 
//             after starting the autopilot gets ignored by using the global variable 
//             ignoreFirstValue.
//
// stopping() stops the motors of the robot
//
// reardriving() the robot starts driving backwards 
//
// turnLeft()  the robot turns left
//
// turnRight() the robot turns right
//
// lifting ()  the robot grabs the object and raises the arm 
//
// dropping () the robot drops the object and lowers the arm
//
//------------------------------------------------------------------------------------------

package remoteControl;

//imports
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.NXTTouchSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.utility.Delay;


public class InfraredSignalCheckerThread extends Thread {
  
  
    //class variables for the motors and sensors
    private EV3IRSensor infraredSensor;
    private NXTTouchSensor leftTouch, rightTouch;
    private RegulatedMotor leftMotor,rightMotor,armMotor;

    
    //class variable for ignoring the first distance value 
    // of the infrared sensor
    private boolean ignoreFirstValue;
    
 
    //get set to true as long as the current thread is running
    public boolean isRunning;
    
    
    //constructor
    public InfraredSignalCheckerThread(final EV3IRSensor infraredSensor, 
        NXTTouchSensor leftTouch, NXTTouchSensor rightTouch, RegulatedMotor left, 
        RegulatedMotor right, RegulatedMotor arm){
        this.infraredSensor = infraredSensor;
        this.leftTouch = leftTouch;
        this.rightTouch = rightTouch;
        this.leftMotor = left;
        this.rightMotor = right;
        this.armMotor = arm;
    }

    
    //-------------------------------------------------------
    //run ()
    //  reads the input from the remote control and allows 
    //  the robot to drive in different directions and starts 
    //  the self-driving mode depending on the provided input.
    //
    // Effect:
    //  the robot is moved in a certain direction or starts
    //  the self-driving mode
    //-------------------------------------------------------    
    @Override
    public void run() {
      System.out.println("Bereit...");
        
      
      while(isRunning){           
            
            // fetching the command number from the different channels
            final int remoteCommand_1 = infraredSensor.getRemoteCommand(0);
            final int remoteCommand_4 = infraredSensor.getRemoteCommand(3);
            
            // If Statement for commands from channel one
            if((remoteCommand_1 >= 0 && remoteCommand_1 <= 9) && remoteCommand_4 == 0) {
                
                //SwitchCase which allows the robot to drive forward, backwards, left, right
                switch (remoteCommand_1){
                    
                    // no button pressed
                    case 0:
                      leftMotor.startSynchronization();     
                      leftMotor.stop();
                      rightMotor.stop();
                      leftMotor.endSynchronization();
                      break;
                      
                    // turn left  
                    case 1:
                      customDrivingPace(350, 300);
                      rightMotor.forward();
                      break;
                        
                    // turn right   
                    case 2:
                      customDrivingPace(350, 300);
                      leftMotor.forward();
                      break;
                        
                    // drive forward    
                    case 3:
                      customDrivingPace(600, 600);
                      leftMotor.startSynchronization(); 
                      leftMotor.forward();                    
                      rightMotor.forward();
                      leftMotor.endSynchronization();
                      break;
                      
                    // drive backwards  
                    case 4:
                      customDrivingPace(600, 500);
                      leftMotor.startSynchronization();     
                      leftMotor.backward();
                      rightMotor.backward();
                      leftMotor.endSynchronization();
                      break;
                      
                    // dealing with occasional errors  
                    default:
                      System.out.println("Button undefined, continue!");                 
                }
            }
            
            
            // If Statement for commands from channel four
            if ((remoteCommand_4 >= 1 && remoteCommand_4 <= 9) && remoteCommand_1 == 0){
                System.out.println("Command_3 " + remoteCommand_4);
                
                //SwitchCase which allows the robot to turn into self driving mode
                switch (remoteCommand_4){
                                   
                // no button pressed                                           
                case 0:
                  leftMotor.startSynchronization();     
                  leftMotor.stop();
                  rightMotor.stop();
                  leftMotor.endSynchronization();
                  break;
                
                  // self-driving mode 1
                  // remote press to start
                  // robot dodges to the left 
                  case 2:
                    drive(true);                     
                    break;       
                
                
                    // self-driving mode 2
                    // remote press to start
                    // robot dodges to the right 
                   case 4:
                     drive(false);                     
                     break;
                      
                        
                   default:
                      System.out.println("Button undefined, continue!");
                }
            }                                
            
        }
    }
                 
    
    //-------------------------------------------------------
    //drive (boolean dodgeDirection)
    //  Allows the robot to drive by himself using autopilot-
    //  mode. The robot detects table edges by himself due
    //  to the mounted touch sensors on both sides. He is 
    //  also able to dodge barriers by using the infrared
    //  sensors. While using his third motor the robot is
    //  capable of picking up objects and dropping them
    //  at the desired location. 
    //
    // Parameter: needs a boolean value to operate
    //
    // Effect:
    //  the robot drives with autopilot and deals with the
    //  tasks of removing objects from the table.
    //-------------------------------------------------------  
    public void drive (boolean dodgeDirection) {
 
      ignoreFirstValue = true;                     
      customArmPace(200, 230);
      lifting(); 
            
      
       t: while(isRunning) {

                
          // fetching commands of the left touch sensor
          float[] sampleL = fetchLeft();
          
          //fetching commands of the right touch sensor
          float[] sampleR = fetchRight();
          
          boolean hindernis = obstacle();
          
          // Case 1 right and left pressed
          // driving straight forward and scanning for barriers
          if ((sampleL[0] == 1 && sampleR[0] == 1) && (hindernis == false)) {
           
              customDrivingPace(270, 220);
              
              leftMotor.startSynchronization(); 
              leftMotor.forward();
              rightMotor.forward();
              leftMotor.endSynchronization(); 
             
          }
          
          
       // drives into barrier on the left side and detects barrier with infrared sensor at the same time
          else if((sampleL[0] == 0 && sampleR[0] == 1) && (hindernis == true)) {
             reardriving(1000);   
             stopping();
             if((sampleL[0] == 1 && sampleR[0] == 1) && (hindernis == false)) {
               turnRight(600);
             }
             else {
               reardriving(400);
             }
             stopping();
          }
          
          
          // drives into barrier on the right side and detects barrier with infrared sensor at the same time
          else if((sampleL[0] == 1 && sampleR[0] == 0) && (hindernis == true)) {
            reardriving(1000);      
            stopping();
            if((sampleL[0] == 1 && sampleR[0] == 1) && (hindernis == false)) {
              turnLeft(600);
            }
            else {
              reardriving(400);
            }
            stopping();
          }
                          
          // driving straight into a barrier, infrared sensor detects barrier
          else if( (sampleL[0] == 1 && sampleR[0] == 1) && (hindernis == true)) {

            
            stopping();
            System.out.println("Infrared Barrier");
            reardriving(1600);                                                    
            
            if(dodgeDirection) {
               turnLeft(1750);
            }
            else {
              turnRight(1750);
            }
          }
          
          // driving off the table 
          // one or both of the sensors are not pressed anymore
          else if((sampleL[0] == 0 || sampleR[0] == 0) && (hindernis == false)) {
           
            System.out.println("Left: " +sampleL[0] +", Right: " +sampleR[0]);
        
            reardriving(1600);                                
            
            float [] oldSampleL = sampleL;
            float [] oldSampleR = sampleR;
            
            // fetching commands left-touch --> could have changed
            sampleL = fetchLeft();
            
            //fetching commands right-touch --> could have changed
            sampleR = fetchRight();

                           
            // Case 1: right sensor pressed, left not           
            if(sampleL[0] == 0 && sampleR[0] == 1) {
             
              System.out.println("Case1");
              stopping();
              
              // both sensors done, due to rolling after stopping
              if(sampleL[0] == 0 && sampleR[0] == 0) {
                dropping();
                reardriving(500);
                break t;
              }
                                        
              customDrivingPace(220, 200);
              reardriving(700);
              turnLeft(1000); 
              stopping();
              
              
              // driving forward after the turn 
              driveForwardAfterTurn();   
              
              // self driving mode exit
              break t; 
              
            }
                                 
            // Case 2: left sensor pressed, right not 
            else if(sampleL[0] == 1 && sampleR[0] == 0) {
              
              System.out.println("Case2");

              stopping();
              
              // both sensors down, due to rolling after stopping
              if(sampleL[0] == 0 && sampleR[0] == 0) {
                dropping();
                reardriving(500);
                break t;
              }
              
              customDrivingPace(220, 200);
              reardriving(700);
              turnRight(1000); 
              stopping();
              
              // driving forward after turning
              driveForwardAfterTurn();
              
              // self driving mode exit
              break t; 
              
            }
                        
            // Case 3: left and right not pressed 
             else if(sampleL[0] == 0 && sampleR[0] == 0) {
               
               System.out.println("Case3");

               stopping();
               
               dropping();
               System.out.println("Dropping brick !");
               
               customDrivingPace(700, 300);
               reardriving(500);
               stopping();
               
               // self driving mode exit
               break t; 
                                      
             } 
            
             // Case 4: barrier detected
             else { 
               System.out.println("Barrier !");
               reardriving(1000);           
               stopping();
               
               // checking the turn direction depending on the old sample 
               if(oldSampleL[0] == 0 && oldSampleR[0] == 1){
                 turnLeft(2500);          
               }
               else if(oldSampleL[0] == 1 && oldSampleR[0] == 0){
                 turnRight(2500);          
               }
               else {
                 turnLeft(2500);      
               }
               stopping();
             }
          }
          
    
       }
      System.out.println("Waiting for Button press!");

    }
    
    
    
    //-------------------------------------------------------
    //customDrivingPace(int motorSpeed, int motorAccelaration)
    //  setting driving speed and accelaration
    //
    // Parameters
    //  needs two integers between 0 and 700 to operate
    //
    // Effect:
    //  the driving speed and accelaration get set
    //-------------------------------------------------------
    public void customDrivingPace (int motorSpeed, int motorAccelaration) {
      leftMotor.setSpeed(motorSpeed);
      rightMotor.setSpeed(motorSpeed);
      
      leftMotor.setAcceleration(motorAccelaration);
      rightMotor.setAcceleration(motorAccelaration);
          
    }
    
        
    //-------------------------------------------------------
    //customArmPace (int armSpeed, int armAccelaration)
    //  setting arm speed and accelaration
    //
    // Parameters
    //  needs two integers between 0 and 700 to operate
    //
    // Effect:
    //  the  speed and accelaration of the arm gets set
    //-------------------------------------------------------
    public void customArmPace (int armSpeed, int armAccelaration) {
      armMotor.setSpeed(armSpeed);
      armMotor.setAcceleration(armAccelaration);
      
    }
        
    
    
    //-------------------------------------------------------
    //driveForwardAfterTurn ()
    //  the robot drives forward until he reaches the table 
    //  edge for the second time, drops the lego brick and 
    //  drives backward
    //
    // Effect:
    //  the robot drives forward until he reaches the table 
    //  edge for the second time, drops the lego brick
    //  and drives backward
    //-------------------------------------------------------
    public void driveForwardAfterTurn () {  
            
      while(isRunning) {
              
        // fetching commands left
        float [] sampleL2 = fetchLeft();

        // fetching commands right
        float[] sampleR2 = fetchRight();

        // driving forward
        leftMotor.startSynchronization();           
        leftMotor.forward();
        rightMotor.forward();
        leftMotor.endSynchronization(); 
        
        // checking if sensors aren't pressed anymore
        if(sampleL2[0] == 0 || sampleR2[0] == 0) {    
          stopping();
          
          dropping();
          
          System.out.println("Dropping brick !");
          
          customDrivingPace(700,300);
          reardriving(500);
          stopping();
          break;
        }
      }
      
    }
    

    
    
    //-------------------------------------------------------
    //fetchLeft ()
    //  gets the signals from the left touch sensor
    //  
    // Effect:
    //  returns the signals of the left touch sensor
    //-------------------------------------------------------
    public float[] fetchLeft() {
      
      int sampleSizeL = leftTouch.sampleSize();
      float[] sampleL = new float[sampleSizeL];
      leftTouch.fetchSample(sampleL, 0);
      
      return sampleL;
    }
    
    
    //-------------------------------------------------------
    //fetchRight ()
    //  gets the signals from the right touch sensor
    //  
    // Effect:
    //  returns the signals of the right touch sensor
    //-------------------------------------------------------
    public float[] fetchRight() {
      
      int sampleSizeR = rightTouch.sampleSize();
      float[] sampleR = new float[sampleSizeR];
      rightTouch.fetchSample(sampleR, 0);
      
      return sampleR;
    }
    
 
    
    //------------------------------------------------------- 
    //obstacle ()
    //  checks for Barriers in front of the robot. 
    //  If the value is smaller than a specified amount,
    //  the robot returns true if not false.
    //  The first measurement after starting the autopilot
    //  gets ignored by using the global variable 
    //  ignoreFirstValue.
    //
    // Effect:
    //  checks for Barriers in front  of the robot and returns
    //  true if an obstacle got detected and false if not.
    //-------------------------------------------------------
    public boolean obstacle() {
      
      SampleProvider distance = infraredSensor.getDistanceMode();  
      
      //gives the average of the last 5 samples
      SampleProvider average = new MeanFilter(distance,5);

       float[] sample = new float[average.sampleSize()];
       average.fetchSample(sample, 0);
       
       // Printing the distance 
       System.out.println("Current: " + sample[0]);                                             

       if (sample[0] < 7 && ignoreFirstValue == false) {
         
         ignoreFirstValue = true;
         System.out.println("Obstacle true!");
         return true;      
       }
       else {
         ignoreFirstValue = false;
         return false;
       }

    }
           
    
    //-------------------------------------------------------
    //stopping ()
    //  stops the motors of the robot 
    //
    // Effect:
    //  the robot gets stopped
    //-------------------------------------------------------
    public void stopping() {
      leftMotor.startSynchronization();           
      leftMotor.stop();
      rightMotor.stop(); 
      leftMotor.endSynchronization();
      
      System.out.println("stopping");
    }

    
    //-------------------------------------------------------
    //reardriving(int duration)
    //  the robot starts driving backwards for a certain amount
    //  of time depending on the parameter
    //
    // Parameter: needs a positive integer value to operate
    //
    // Effect:
    //  the robot starts driving backwards for a certain amount
    //  of time depending on the parameter
    //-------------------------------------------------------
    public void reardriving(int duration) {
      leftMotor.startSynchronization();           
      leftMotor.backward();
      rightMotor.backward();  
      leftMotor.endSynchronization();
      Delay.msDelay(duration);                                                          

      System.out.println("Reardriving");
    }
    
    
    //-------------------------------------------------------
    //turnLeft (int duration)
    //  the robot turns left for a certain amount
    //  of time depending on the parameter                                                
    //
    // Parameter: needs a positive integer value to operate
    //
    // Effect:
    //  the robot turns in the direction on the left hand 
    //  side
    //-------------------------------------------------------
    public void turnLeft(int duration) {
      rightMotor.forward();
      System.out.println("Left");
      Delay.msDelay(duration);
    }                                             
    
    
    //-------------------------------------------------------
    //turnRight ()                                                                 
    //  the robot turns right for a certain amount
    //  of time depending on the parameter  
    //
    // Parameter: needs a positive integer value to operate
    //
    // Effect:
    //  the robot turns in the direction on the right hand 
    //  side
    //-------------------------------------------------------
    public void turnRight(int duration) {
      leftMotor.forward();
      System.out.println("Right");
      Delay.msDelay(duration);
    }
    
       
    
    //-------------------------------------------------------
    //lifting ()                                                                 
    //  the robot grabs the object and raises the arm 
    //
    // Effect:
    //  the robot grabs the object and raises the arm 
    //  by using the third motor, the arm motor.
    //-------------------------------------------------------
    public void lifting() {
      armMotor.backward();
      Delay.msDelay(1500);
      armMotor.stop();
    }
    
    
    //-------------------------------------------------------
    //dropping ()                                                                 
    //  the robot drops the object and lowers the arm 
    //
    // Effect:
    //  the robot drops the object and lowers the arm 
    //  by using the third motor, the arm motor.
    //-------------------------------------------------------
    public void dropping() {
      armMotor.forward();
      Delay.msDelay(1500);
      armMotor.stop();
    }
    
 
}



