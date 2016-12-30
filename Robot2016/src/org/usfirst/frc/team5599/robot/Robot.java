// Main Code

// wpi Library: http://first.wpi.edu/FRC/roborio/release/docs/java/


//==================================================================================================================\\
//==[[ Robot.java ]]================================================================================================\\
//==================================================================================================================\\


//          Robot Driving
package org.usfirst.frc.team5599.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotDrive; 
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.BuiltInAccelerometer; // Roborio comes in with a built-in accelerometer
// import edu.wpi.first.wpilibj.AnalogGyro; // But not a gyro. According to Tanoy it came with a free external one.
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Joystick.RumbleType;

//          Talons, Victors, Solenoids, Compressors
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
// import edu.wpi.first.wpilibj.Solenoid;       //Pneumatics
import edu.wpi.first.wpilibj.DoubleSolenoid;    //Pneumatics
import edu.wpi.first.wpilibj.Compressor;     //Pneumatics

//          Timer, Server
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.CameraServer;
// import edu.wpi.first.wpilibj.vision.USBCamera


public class Robot extends SampleRobot {

    // Wheels
    Victor frontLeft;
    Victor rearLeft;
    Talon frontRight;
    Talon rearRight;

    // Other motors
    Victor intakeMotor; // Controls gears at front that pull in the ball //Valerie
    // Victor pulleyMotor; // Pulley motor 
    Victor shootingMotor; // Shooting motor
    Victor portyMotor; // Chaire De Fraux motor arm
    // Victor cameraMotor;

    DoubleSolenoid liftArm;
    Compressor liftCompressor;

    RobotDrive myRobot;  // class that handles basic drive operations
    JoystickController testpad;
    XBoxController gamepad; // declare gamepad as a Joystick object to get Joystick functionality, but with different buttons
    XBoxController gamepad2;


    // AnalogGyro rGyro;
    BuiltInAccelerometer rAccel;

    // CameraServer server; // Camera server class


    //Pneumatics info: http://www.chiefdelphi.com/forums/showthread.php?t=91938 // Wtf is this for someone go check later

    
    /*

    // Old robot code for Garbage dumper thing

    DoubleSolenoid pistonSqueeze;
    Solenoid pistonLift;
    Compressor compressor;

    */

    // Long ass string ;-;
    String[] ControlsString = { "-CONTROLS- ",
        "Controller 1:",
        "> Joysticks - Manuevering ",
        "> Right Trigger - Run Porty Motor",
        "> Left Trigger - Run Porty Motor Inverted",
        "> Right Bumper - Run New Motor",
        "> Left Bumper - Run New Motor Inverted",
        "",
        "Controller 2:",
        "> Right Trigger - Run Shooting Motor",
        "> Left Trigger - Run Shooting Motor Inverted",
        "> Right Bumper - Run Intake Motor",
        "> Left Bumper - Run Intake Motor Inverted",
        "> DPad Up - Raise Arm",
        "> DPad Down - Lower Arm",
        "",
        "X - Print Controls ",
        ""
    };
    
    public Robot() {

        print("Robot Enabled");

        frontLeft = new Victor(0); //change port to appropriate PWM port on roboRIO
        rearLeft = new Victor(1); //change port to appropriate PWM port on roboRIO
        frontRight = new Talon(2); //change port to appropriate PWM port on roboRIO
        rearRight = new Talon(3); //change port to appropriate PWM port on roboRIO
        
        /*

        .set usually sets the speed. This was uncommented before 2/19 and I'm not sure why we're setting the motors to run at this speed right now
            - Michael

        (According to the wpilib it sets the PWM, I wonder if this presets it so that when we use it doesn't go past .70?)

        */

        frontLeft.set(1.00);
        frontRight.set(0.95);
        rearLeft.set(1.00);
        rearRight.set(0.95);

        print("Wheels initalized");

        // Operating our motors & solenoids
        intakeMotor = new Victor(4);        // RoboRio port for this motor goes here
        // pulleyMotor = new Victor(7);        // RoboRio port for this motor goes here
        shootingMotor = new Victor(5);      // RoboRio port for this motor goes here
        portyMotor = new Victor(6);
        // cameraMotor = new Victor (7);

        print("Victors (Intake, Shooting, Porty) initalized");

        liftArm = new DoubleSolenoid(1,3);  // Set to appropriate solenoid port, channel
        liftCompressor = new Compressor(); // Create a new Compressor virtual object

        print("Arm compressor and solenoid initalized");

        // Setting up the robot and driving controls
        myRobot = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
            // robot drivetrain functionality; takes in front left motor, rear left motor, front right motor, rear right motor ports
        
        
        gamepad = new XBoxController(0); // port controller 1 is plugged into
        gamepad2 = new XBoxController(1); // port controller 2 is plugged into
        // The XBoxController class is found in the XBoxController.java file under src
            // That class also has a buffer (As Chief Delphi said we'd need) on the joysticks.
            // This protects against extremely sensitivity, where the slightest touch can send the robot just driving
        testpad = new JoystickController(2); // For attemptin to make the Logitech Joystick work on the robot.
            // Which is a terrible idea considering we use the Tank Drive model and the joystick is for Arcade Drive.

            // You can re-order the USB numbers in the Driver Station

        print("Controllers initalized");

        // Gyro tips: https://wpilib.screenstepslive.com/s/3120/m/7912/l/85772-gyros-to-control-robot-driving-direction
        // rGyro = new AnalogGyro(0); // Gyro port on 'Analog Channels'
        rAccel = new BuiltInAccelerometer();
        // rAccel = new BuiltInAccelerometer(Accelerometer.Range.k4G); // Sets how many Gs we measure in

        // rGyro.calibrate();

        // Operating the Camera
        // server = CameraServer.getInstance();
        // server.setQuality(30);
        // server.startAutomaticCapture("cam0"); // The camera name (ex "cam0") can be found through the roborio web interface

        /*
        compressor = new Compressor();  //Set to null
        pistonLift = new Solenoid(2);   //Set to appropriate solenoid port, channel
        pistonSqueeze = new DoubleSolenoid(0, 1);   //Set to appropriate solenoid port, channel
        */

    }
    
    // 2 Camera Setup (Lifecam HD 300)
    // functions

    public void print(String message) {
        // Custom print statement because System.out.println(); is too tedious!
        System.out.println(message);
    }
    
    public void printDouble(double Double) {
        // Can't send a double to the first print statement because it only works with strings
        System.out.println(Double);
    }
    
    public void printInt(int Integer) {
        // Can't send an integer to the first print statement because it only works with strings
        System.out.println(Integer);
    }

    public void StopIntakeMotor() {
        // Seperate function just for autonomous
        intakeMotor.set(0);
        gamepad.setRumble("Both", 0); 
        gamepad2.setRumble("Both", 0); 
    }

    public void OperatePorty(){
        if (gamepad2.getRightTrigger() == true) {
            portyMotor.set(1.0);
            print("G2 - Right Trigger - Running Porty Motor");
        }
        else if (gamepad2.getLeftTrigger() == true) {
            portyMotor.set(-1.0);
            print("G2 - Left Trigger - Running Porty Motor Inversed");
       }
        else {
            portyMotor.set(0);
        }
    }

    public void OperateCameraMotor(){
        /*
        if (gamepad.getDPadLeft() == true) {
            cameraMotor.set(0.01);
            print("G1 - Right Bumper - Running Camera Motor");
        }
        else if (gamepad.getDPadRight() == true) {
            cameraMotor.set(-0.01);
            print("G1 - Left Bumper - Running Camera Motor Inversed");
       }
        else {
            cameraMotor.set(0);
        }
        */
    }

    public void OperateIntake(boolean AutoIntake) {
        // Controls the intake Motor

        if ((gamepad.getRightTrigger() == true) || (AutoIntake == true)) {
            intakeMotor.set(1.0);
            if (AutoIntake == true) {
                print("Autonomous Mode - Running intakeMotor");
            } 
            else {
                print("G1 - Right Trigger - Running Intake Motor");
                // gamepad.setRumble("Both", 1);
            }
        }
        else if (gamepad.getLeftTrigger() == true) {
            intakeMotor.set(-1.0);
            print("G1 - Left Trigger - Running Intake Motor Reversed");
            // gamepad2.setRumble("Both", 1);
        } 
        else {
            StopIntakeMotor();
        }

    }

    public void OperateShooter(boolean AutoShoot) {

        if ((gamepad2.getBButton() == true) || (AutoShoot == true)) {
            shootingMotor.set(-0.9);
            if (AutoShoot == true) {
                print("Autonomous Mode - Charging shootingMotor");
            } 
            else {
                print("G2 - B Button - Running ShootingMotor");
               // gamepad.setRumble("Both", 1);
            }
        }
        else if (gamepad2.getAButton() == true) {
            shootingMotor.set(0.9);
            print("G2 - A Button - Running ShootingMotor Inversed");
        }
        else {
            shootingMotor.set(0);
           // gamepad.setRumble("Both", 0};
        }

    }

    public boolean isCruiseControl = false;

    public void ToggleCruiseControl(boolean cruiseControl){
        isCruiseControl = cruiseControl;
    }

    public void OperateLift() {

        liftCompressor.start();
        liftCompressor.setClosedLoopControl(true); //Lets PCM handle the automatic turning on and off of compressor once pressure hits 120 psi  

        // Lift le robooot
        if (gamepad2.getRightBumper() == true) {
            liftArm.set(DoubleSolenoid.Value.kReverse);
            print("G2 - Right Bumper - Lowering Arm");
        }
        else if (gamepad2.getLeftBumper() == true) {
            liftArm.set(DoubleSolenoid.Value.kForward);
            print("G2 - Left Bumper - Raising Arm");
        }
        else {
            liftArm.set(DoubleSolenoid.Value.kOff);
        } 


    }

    // Compressor info before we go boom
    public void CheckCompressor() {
        double current = liftCompressor.getCompressorCurrent();
        SmartDashboard.putNumber("Current", current);
        boolean tooHigh = liftCompressor.getCompressorCurrentTooHighFault();
        SmartDashboard.putBoolean("Current is too high", tooHigh);
        boolean notConnected = liftCompressor.getCompressorNotConnectedFault();
        SmartDashboard.putBoolean("Current is not Connected", notConnected);
        if (tooHigh) {
            print("Compresor current too high");
        }
        if (notConnected) {
            print("Compressor current not connected");
        }
    }

    public void CheckDPad() {
        /*
        if (gamepad.getDPadUp() == true) {
            print("DPadUp is down!");
        }
        if (gamepad.getDPadDown() == true) {
            print("DPadDown is down!");
        }
        if (gamepad.getDPadLeft() == true) {
            print("DPadLeft is down!");
        }
        if (gamepad.getDPadRight() == true) {
            print("DPadRight is down!");
        }
        */

    }

    public void PrintControls(boolean Override) {
        /*
        if ((gamepad.getStartButton() == true) || (gamepad2.getStartButton() == true) || (Override == true)) {
            for( int i = 0; i <= ControlsString.length - 1; i++) {
                print(ControlsString[i]);
            }
            if (Override != true) {
                Timer.delay(0.1);
            }
        }
        */
    }

    public void RobotBase() {//Runs before anything else
        //DriverStation();
    }

    public void DriverStation() { //Takes input from Driver Station itself
        DriverStation ds = DriverStation.getInstance();
        DriverStation.Alliance color;
        color = ds.getAlliance();
        String allianceColor;
        if (color == DriverStation.Alliance.Blue) {
            allianceColor = "Blue";
        }
        else if (color == DriverStation.Alliance.Red) {
            allianceColor = "Red";
        }
        else {
            allianceColor = "Invalid";
        }
        double voltage = ds.getBatteryVoltage();
        boolean brown = ds.isBrownedOut();
        SmartDashboard.putString("Alliance Color", allianceColor);
        SmartDashboard.putNumber("Voltage", voltage);
        SmartDashboard.putBoolean("Brown Out", brown);
        if (brown) {
            print("Experiencing brownout");
        }
    }
    
    /*
     * Runs the motors with tank steering.
     */


    public void autonomous() { // This is Autonomous

        // How to set up autonomous: 
        // https://wpilib.screenstepslive.com/s/4485/m/24192/l/144977-frc-driver-station-labview-dashboard
        // https://wpilib.screenstepslive.com/s/4485/m/24192/l/272692?data-resolve-url=true&data-manual-id=24192

        /*
            Shooting:
            OperateShooter(true) // Run the motor, wait, stop the motor (Hopefully after the ball has been fired)
    
            Intake:

            OperateIntakeMotor(true) // Only runs the motor
            StopIntakeMotor() // Stops the motor
        */

        // This has some more cool autonomous code
        // http://first.wpi.edu/Images/CMS/First/GettingStartedWithJava.pdf
        // https://wpilib.screenstepslive.com/s/4485/m/13809/l/241857-getting-your-robot-to-drive-with-the-robotdrive-class

        /*
        print("Resetting Gyro");
        rGyro.reset();
        print("Gyro Reset");
        */

        print("Robot Entered Autonomous Mode");

        /*
        

        int tickCount = 0;
        
        while (isAutonomous() && isEnabled()) {
            DriverStation();
            if ((ticketCount < 2000) && (isAutonomous()) & (isEnabled())) {
                myRobot.drive(-0.5, 0.0);
                ticketCount++;
                printInt(ticketCount);
            }
            else {
                print("Robot stopped accordingly");
                myRobot.drive(0.0, 0.0);
            }
        }
        */

        print("Autonomous Mode Ended");
        myRobot.drive(0.0, 0.0);

    }

    public void operatorControl() { // This is tele-op

        // PrintControls(true);
        print("Tele-op mode started");

        myRobot.setSafetyEnabled(false); //Safety Controls: On or Off
            /*
                What Safety is:
                http://www.chiefdelphi.com/forums/showthread.php?t=105538
                https://wpilib.screenstepslive.com/s/4485/m/13809/l/241863-using-the-motor-safety-feature
                    
                    Someone should read this as I have definitely not - Mike
            */

        while (isOperatorControl() && isEnabled()) {

            if (isCruiseControl == false){
                myRobot.tankDrive(gamepad.getRightThumbstickY(), gamepad.getLeftThumbstickY(), true); //Get the Controls
            }
           else {
                myRobot.tankDrive(0.2, 0.2, true); //Cruise at a nice .2 speed
           }


            // UseCorrectController();gvv. 
            // PrintControls(false);

            Timer.delay(0.005); // wait for a motor update time 

            DriverStation();

            OperateIntake(false);
            OperateShooter(false);
            OperatePorty();
            // OperateCameraMotor();
            // CheckDPad();
            CheckCompressor();
            OperateLift();
        }
    }

    public void disabled() { // When the robot is disabled from the Driver Station, make sure the robot actually stops
        print("Disabling robot . . .");
        myRobot.drive(0,0);
        print("Robot Drive Stopped");
        intakeMotor.set(0);
        print("Intake Motor Stopped");
        shootingMotor.set(0);
        print("Shooting Motor Stopped");
        portyMotor.set(0);
        print("Porty Motor Stopped");
        liftCompressor.stop();
        print("Lift Compressor Stopped");
        liftArm.set(DoubleSolenoid.Value.kOff);
        print("Lift Arm Solenoids Off");
        gamepad.setRumble("Both", 0); 
        gamepad2.setRumble("Both", 0);
        print("Gamepad Rumble Turned off");
        print("Robot's disabled() event is complete");
    }

    public void test() {

        while (isTest() && isEnabled()) {

            myRobot.tankDrive(testpad.getJoystickY(), testpad.getJoystickY(), true); //Get the Controls

            if (testpad.getButtonOne() == true) {
                print("button one pressed");
            }

        }

        /*

        String[] rsp = { "What do you want?", "There's nothing here.", 
            "Isn't this a waste of battery?", "Please do something else.", 
            "Why.", "Whhyyy?", "Okay sure. No problem. Have it your way.",
            "Holy christ turn it off already!"
        };

        print("Robot is in Test Mode!");
        print("It doesn't really do anything in test mode.");

        Timer.delay(1);

        print("Are you expecting something???");

        while (isTest() && isEnabled()) {
            for( int i = 0; i <= rsp.length - 1; i++) {
                print(rsp[i]);
                Timer.delay(4);
            }
            Timer.delay(0.005); // Just incase I fuck something up and we get a stack overflow o_o
        }

        */

    }

}

/*
Benjamin N. Cardozo Highschool Robotics
    Team 5599

    Hritik Kumar
    Michael Rooplall
    Valerie Macias
    Jeremy Gangaram
    Katie Tam


10/27/15 -- present

*/