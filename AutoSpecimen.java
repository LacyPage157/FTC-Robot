package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.concurrent.TimeUnit;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.lang.Math;


@Autonomous(name="AutoSpecimen", group="Robot")
public class AutoSpecimen extends LinearOpMode
{

    //Hardware Decleration Section
    public DcMotor  leftDrive   = null; //the left drivetrain motor
    public DcMotor  rightDrive  = null; //the right drivetrain motor
    public DcMotor  armMotor    = null; //the arm motor
    public DcMotor  armTwo    = null; //the arm motor
    public CRServo  intake      = null; //the active intake servo
    public Servo    wrist       = null; //the wrist servo
    public IMU imu;
    //End Hardware Decleration Section

    //Start April Tag Section
    final double DESIRED_DISTANCE = 24.0; //  this is how close the camera should get to the target (inches)
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private float robRange;
    private float robBearing;
    private float robYaw;
    //END APRIL TAG SECTION


    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    //DOWN

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    double CountsPerRev = 541; //mber of encoder ticks per rotation of the bare motor with down below
    // 28* (250047.0 / 4913.0) // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
    double WheelDiameterInches = 3.77953;     // For figuring out circumference
    double CountsPerInch = ((CountsPerRev / (WheelDiameterInches * 3.1415926535))); 

    //Navigation basic variables
    public double turningSpeed = .4;
    public int forward = 1;
    public int reverse = -1;
    public double left;
    public double right;
    public double rotate;
    public double max;

    double Yaw; //NOTE: HEADING IS YAW!!!! THEY ARE THE SAME THING!!! 
    double Pitch;
    double Roll;

    double programStartOrientation;
    public double stayOnHeading = 84.17; //Arbitrary number decided more or less...

    public YawPitchRollAngles robotOrientation;
        

    public ElapsedTime runtime = new ElapsedTime(); //This will give us the time since we have started, very useful for timing things.
    
    @Override
    public void runOpMode() {

        telemetry.addData("Status:", "Begin Init");
        telemetry.update();




        //Hardware Map declerations matching

        /* Define and Initialize Motors */
        leftDrive  = hardwareMap.get(DcMotor.class, "rightMotor"); //the left drivetrain motor
        rightDrive = hardwareMap.get(DcMotor.class, "leftMotor"); //the right drivetrain motor
        armMotor   = hardwareMap.get(DcMotor.class, "armOne"); //the arm main motor
        armTwo = hardwareMap.get(DcMotor.class, "armTwo"); //the sub arm motor
        imu = hardwareMap.get(IMU.class, "imu");

        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");
        

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);


        //Start IMU Section//

        //Here, we create and initilize the IMU -Basically an integrated sensor
        imu.initialize(
        new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, 
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT //This decides axis relative to our Robot....
            ) //Make sure this is accurate ^ !!!!
        )
        );
        

        
        robotOrientation = imu.getRobotYawPitchRollAngles();

        // Now use these simple methods to extract each angle
        // (Java type double) from the object you just created:
        Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES); //NOTE: HEADING IS YAW!!!! THEY ARE THE SAME THING!!! 
        Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

        AngularVelocity myRobotAngularVelocity;

        // Read Angular Velocities
        myRobotAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        // Then read or display these values (Java type float)
        // from the object you just created:
        float zRotationRate = myRobotAngularVelocity.zRotationRate;
        float xRotationRate = myRobotAngularVelocity.xRotationRate;
        float yRotationRate = myRobotAngularVelocity.yRotationRate;

        //End IMU Section//

        // Initialize the Apriltag Detection process
        initAprilTag();




        /* Send telemetry message to signify robot waiting */
        telemetry.addData("Status:", "Robot Ready");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();

        //We will build the rest of this once the other functions are made...















        //Left empty on purpose

        // public void encoderDriveSmooth(double speed, double Inches, int direction, double heading, double timeout) { 

        //encoderDriveSmooth(5.0,24.0,forward,0,3000); //move forward 24 inches
        telemetry.addData("Byleth","Byleth is cool");
        telemetry.update();
        if (!leftDrive.isBusy()){
            // SimpleEncoderDrive(0.5,24.0,forward);
            // sleep(1000);
            // SimpleEncoderDrive(0.5,12.0,reverse);
            // sleep(1000);
            // SimpleEncoderDrive(0.5,24.0,forward);
            
            //FastEncoderDrive(20, forward);
            // SimpleEncoderDrive(0.5,8,reverse);
            // SimpleEncoderTurn(0.7, 179);
            // armUp(false);
            // FastEncoderDrive(15,forward);
            // FastEncoderDrive(6,reverse);
            // armUp(false);
            // SimpleEncoderTurn(0.5,-90);
            // SimpleEncoderDrive(0.5,30,forward);
            // SimpleEncoderTurn(1,90);
            // SimpleEncoderDrive(0.5,30,forward);
            // SimpleEncoderTurn(1,-90);
            
            SimpleEncoderDrive(0.5,8,reverse, 5);
            SimpleEncoderTurn(0.5, -179,6);
            armClip(false,0.8);
            SimpleEncoderDrive(0.2,5.0,reverse, 5);
            armUp(true);
            armClip(false,1); //Arm Up right after
            SimpleEncoderDrive(1,21,forward, 5);
            SimpleEncoderDrive(0.5,17,reverse), 5;
            armUp(false);
            SimpleEncoderTurn(0.5,-90,4);
            SimpleEncoderDrive(0.5,30,forward, 5);
            SimpleEncoderTurn(1,90,4);
            SimpleEncoderDrive(0.5,30,forward, 5);
            SimpleEncoderTurn(1,-90,4);
            
            
            
        }
        //moves 2 5/16th inch to the right 

        // encoderDriveSmooth(5.0,24.0,-1,0,5); //move backward 24 inches

        // encoderDriveSmooth(25.0,24,1,130,5); //heading test

        



        //We will build the rest of this once the other functions are made...
    }
    
    public void HardDrive(int s, int direction){
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setPower(0.2);
        leftDrive.setPower(-0.2);
        telemetry.addLine("running");
        telemetry.update();
        sleep(s);
        rightDrive.setPower(0);
        leftDrive.setPower(0);
    
    
    }

    //Arm Position Functions:
    public void armCollect(Boolean pause){
        armPosition = ARM_COLLECT; //Pick up blocks
        wrist.setPosition(WRIST_FOLDED_OUT);
        intake.setPower(INTAKE_COLLECT);

        armMotor.setTargetPosition((int) (armPosition)); //Set the position to an set constant initialized at the start of program

        ((DcMotorEx) armMotor).setVelocity(2100); //Run it at a constant velocity for the sake of accuracy instead of having to deal with modifying powert
        //The RUN_TO_POSITION mode for a DcMotor will automatically adjust the power to get to our target, so we should just adjust speed
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armMotor.isBusy()&& pause){
            telemetry.addData("Current Arm Position:",armMotor.getCurrentPosition()); //Basically hold the program up while things run. 
            telemetry.update();
        
        }
    }


    public void armBucket(Boolean pause){
        armPosition = ARM_SCORE_SAMPLE_IN_LOW; //For use with low bucket -- 
        wrist.setPosition(WRIST_FOLDED_OUT);

        armMotor.setTargetPosition((int) (armPosition));

        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.isBusy() && pause){
            telemetry.addData("Current Arm Position:",armMotor.getCurrentPosition());
            telemetry.update();
        
        }
    }
    public void armUp(Boolean pause){
        armPosition = ARM_ATTACH_HANGING_HOOK; //For use with low bucket -- 
        //wrist.setPosition(WRIST_FOLDED_OUT);

        armMotor.setTargetPosition((int) (armPosition));

        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.isBusy() && pause){
            telemetry.addData("Current Arm Position:",armMotor.getCurrentPosition());
            telemetry.update();
        
        }
    }

    public void setIntake (int power){
        //Power takes -1, 0 or 1 -- -1 is intake collecting, 1 is depositing
        switch(power){
            case -1: 
                intake.setPower(INTAKE_COLLECT);
                break;
            case 0:
                intake.setPower(INTAKE_OFF);
                break;
            case 1:
                intake.setPower(INTAKE_DEPOSIT);
                break;
            default:
                telemetry.addLine("ERROR 1: SetIntake(int) takes type int from -1 to 1.");
                telemetry.addLine("ERROR 1: takes -1, 0 or 1 ... -1 is collecting, 1 is depositing");
                telemetry.update();
                sleep(1000);

        }
    }

    public void armClip(Boolean pause,double factor){
        armPosition = ARM_SCORE_SAMPLE_IN_LOW*factor; //Clip a specimen on high rung
        wrist.setPosition(WRIST_FOLDED_IN);
        intake.setPower(INTAKE_OFF);

        armMotor.setTargetPosition((int) (armPosition));

        ((DcMotorEx) armMotor).setVelocity(5000);
        armMotor.setPower(1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armMotor.isBusy() && pause){
            telemetry.addData("Current Arm Position:",armMotor.getCurrentPosition());
            telemetry.update();
        
        }
    }



    public double getHeading(){
        //returns the Yaw -- Makes it easier so I can just refrence 
        //heading as just that instead of having to think of Yaw, Pitch, Roll etc...
        robotOrientation = imu.getRobotYawPitchRollAngles();
        Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        return Yaw;
    }


    //This section needs work! 
    //Note: Someone will have to clean up these nasty telemetry statements at some point and comments
    public void encoderDriveSmooth(double speed, double Inches, int direction, double heading, double timeout) { 
        /**Speed is how fast we want our robot to go to a set target. NOTE: THIS SHOULD BE A VALUE NORMALIZED 
         * Inches is the distance, in Inches. Positive.
         * Direction should be filled with the forward constant -- Use forward or reverse. 
         * Heading is the degrees we should be going to. For staying straight, use current heading.
         * Timeout will be used at a later date once it is worked out.
        */

       //THIS FUNCTION IS DEPRECATED AND EXISTS FOR HISTORICAL CATALOGING AND FUTURE REFERENCE

        double startTime = runtime.seconds();
        double Heading = 0;
        boolean notAtTarget = true;
        //if we give a very very specific value for our heading, than we stay on our current path
        //otherwise, we get use the gyroDrive to correct to our desired heading
        if (heading == 84.17){
            Heading = getHeading();
        }else{
            Heading = heading;
        }
        double target;
        if (opModeIsActive() ) {
            telemetry.addLine("Testing While Loop Encoder");
            telemetry.update();
            //make sure that the encoder on the front left motor (which, stupidly, is the only motor
            //we use for distance in this function) is reset to 0
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Note, this will be used later. Changes it so that the encoder runs a certain amount of ticks. 
            //find the amount of encoder ticks to travel based off of the Inches var
            target = leftDrive.getCurrentPosition() + (int) (Inches * CountsPerInch * direction);
            //leftDrive.setTargetPosition(target);
            //rightDrive.setTargetPosition(target);
            //while the opmode is still running, and we're not at our target yet, and we haven't timed out
            telemetry.addData("TargetOne", Inches);
            telemetry.addData("TargetTwo", CountsPerInch);
            telemetry.addData("TargetThree", direction);
            
            
            telemetry.addData("NotAtTarget:", notAtTarget);
            
            
            telemetry.addData("Target-Current:",Math.abs(target) - Math.abs(leftDrive.getCurrentPosition()) > 25);
            telemetry.addData("CurrentPos", Math.abs(leftDrive.getCurrentPosition()));
            telemetry.addData("Target pos",Math.abs(target));
            
            telemetry.addData("StartTime:",startTime);
            telemetry.addData("Timeout:",timeout);
            telemetry.addData("runTime:",runtime.seconds());
            if((startTime + timeout > runtime.seconds()))
            {
                telemetry.addLine("StartTimeOut Condition true");
            }
            
            telemetry.addData("Is while true?", opModeIsActive() && notAtTarget && Math.abs(target) - Math.abs(leftDrive.getCurrentPosition()) > 25
                    && (startTime + timeout > runtime.seconds()));
            
            telemetry.update();
            sleep(1000);
            while(opModeIsActive() && notAtTarget && (Math.abs(target) - Math.abs(leftDrive.getCurrentPosition()) > 25 ) && (startTime + timeout > runtime.seconds())) {
                //use gyrodrive to set power to the motors.  We have the Heading Var decied earlier,
                // and speed and direction change base off of speed and direciton given by the user
                

                gyroDrive(Heading, speed, direction);
                //telemetry.addData("While Loop Active?:","Not runtime while loop active");
                telemetry.addData("CurrentPos", leftDrive.getCurrentPosition());
                telemetry.addData("Target pos",Math.abs(target));
                telemetry.update();
                if (Math.abs(target) == Math.abs(leftDrive.getCurrentPosition()) || Math.abs(target) < Math.abs(leftDrive.getCurrentPosition())){
                    notAtTarget = false;
                }
                // if (!((leftDrive.isBusy()) || (rightDrive.isBusy()))){
                //     notAtTarget = false;
                // }
                
                
            }


            rightDrive.setPower(0);
            leftDrive.setPower(0);
            telemetry.addData("CurrentPos", leftDrive.getCurrentPosition());
            telemetry.addData("Target pos",Math.abs(target));
            telemetry.addLine("We should have reached our target position...");
            telemetry.update();
            sleep(10000);
            
        }
    }

    public void SimpleEncoderDrive(double speed, double Inches, double direction, double timeout){

        if (opModeIsActive() ) {
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);

            double startTime = runtime.seconds();
            
            double targetSpeed = 0;


            //Basic dummyproofing
            if ((speed > 1 || speed < 0)){
                telemetry.addLine("ERROR 2: double speed takes values only between 0.0 and 1.0");
                telemetry.update();
                sleep(1000);
            }
            if (Inches < 0){
                telemetry.addLine("ERROR 3: Inches must not be negative -- Change direction to reverse instead!");
                telemetry.update();
                sleep(1000);
            }

        
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setTargetPosition((int)(Inches*CountsPerInch*direction)); //Set the distance according to our set variables.
            //That being, the target position in terms of Encoder ticks so that the motor can read the value. The multiplication
            //changes it from terms we are familiar with to those that the motor is familiar with, in essence. 
            rightDrive.setTargetPosition((int)(Inches*CountsPerInch*direction));
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Basically, run with encoder for a set distance. 
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            if (Inches > 1){
                rightDrive.setPower(0.1); //In essence, give the motors a short push so that the while Loop may activate.
                leftDrive.setPower(0.1);
                sleep(100);
            }
            else {
                telemetry.addLine("ERROR 4: Inches invalid, must be greater than 1!");
                telemetry.update();
                sleep(1000);
            }


            while ((leftDrive.isBusy() || rightDrive.isBusy())&&((startTime + timeout)<runTime.seconds())){

                double error = 1-((double)rightDrive.getCurrentPosition())/((double)rightDrive.getTargetPosition()); //Normalize, then make it 
                //move right to left starting at one, ending at 0.
                double targetVelocity = ((-1/Math.pow(3, ((18*(1.1-speed))*error)))+1)*speed*1000; //Confused? Open desmos and look from 0 to 1, 
                //get rid of * 1000 and speed
                // if(error < 0.01){
                //     leftDrive.setPower(0);
                //     rightDrive.setPower(0);
                //     break;
                // }
                ((DcMotorEx)(rightDrive)).setVelocity(targetVelocity);
                ((DcMotorEx)(leftDrive)).setVelocity(targetVelocity); //Typecast to MotorEx (ie. Motor Premium) 
                // and set to a speed relative to a curve for sake of smoothness.
                telemetry.addData("Running Distance:", Inches);
                telemetry.addData("targetPos",rightDrive.getTargetPosition());
                telemetry.addData("currentPos",rightDrive.getCurrentPosition());
                telemetry.addData("currentVelocity", ((DcMotorEx) rightDrive).getVelocity());
                telemetry.addData("targetVelocity", targetVelocity);
                telemetry.addData("error", error);
                telemetry.update();
                

            }
            // telemetry.addLine("RAHHHHH BRAKING");
            // leftDrive.setPower(0.2*(-speed));
            // rightDrive.setPower(0.2*(-speed));
            // double temp = speed*4*100;
            // sleep((int)temp);
            leftDrive.setPower(0); //Stop the motors
            rightDrive.setPower(0);
            //leftDrive.setPower(0);
            //rightDrive.setPower(0);
            telemetry.addData("Ran Distance:", Inches);
            //telemetry.addData("LongSpeed",(int)temp);
            telemetry.update();
        }

    }
    
    public void FastEncoderDrive(double Inches, double direction){
        
        Inches += 2;

        if (opModeIsActive() ) {
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
            
            double targetSpeed = 0;


            //Basic dummyproofing
            if (Inches < 0){
                Inches = -Inches;
                direction = -direction;
            }
            if (Inches < 1) {
                Inches = 1;
            }
        
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            /*Set the distance according to our set variables.
            That being, the target position in terms of Encoder ticks so that the motor can read the value. The multiplication
            changes it from terms we are familiar with to those that the motor is familiar with, in essence.*/
            leftDrive.setTargetPosition((int)(Inches*CountsPerInch*direction));
            rightDrive.setTargetPosition((int)(Inches*CountsPerInch*direction));
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Basically, run with encoder for a set distance. 
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            
            rightDrive.setPower(0.1); //In essence, give the motors a short push so that the while Loop may activate.
            leftDrive.setPower(0.1);
            sleep(100);
            
            while ((leftDrive.isBusy() || rightDrive.isBusy()) && rightDrive.getCurrentPosition()!=rightDrive.getTargetPosition()){
                //move right to left starting at one, ending at 0. A progress bar
                double error = 1-((double)rightDrive.getCurrentPosition())/((double)rightDrive.getTargetPosition()); //Normalize, then make it 
                double targetVelocity = ((-1/Math.pow(3, ((18*(0.1))*error)))+1)*1300;

                //Typecast to MotorEx (ie. Motor Premium) 
                ((DcMotorEx)(rightDrive)).setVelocity(targetVelocity);
                ((DcMotorEx)(leftDrive)).setVelocity(targetVelocity); 
                
                // and set to a speed relative to a curve for sake of smoothness.
                telemetry.addData("Running Distance:", Inches);
                telemetry.addData("targetPos",rightDrive.getTargetPosition());
                telemetry.addData("currentPos",rightDrive.getCurrentPosition());
                telemetry.addData("currentVelocity", ((DcMotorEx) rightDrive).getVelocity());
                telemetry.addData("targetVelocity", targetVelocity);
                telemetry.addData("error", error);
                telemetry.update();
                

            }
            // stops all motor movement
            leftDrive.setPower(0); //Stop the motors
            rightDrive.setPower(0);
            telemetry.addData("Ran Distance:", Inches);
            telemetry.update();
        }

    }



    
    public void gyroDrive(double targetAngle, double targetSpeed, int direction) {


        //THIS FUNCTION IS DEPRECATED AND EXISTS FOR HISTORICAL CATALOGING AND FUTURE REFERENCE
        
        telemetry.addLine("GyroRunning");
        //telemetry.update();
        //sleep(1000);
        //For use with other functions, but lets us use the gyro to keep the robot on a certain heading
        // it's proportional, so if for instance, a robot hits us, this will account for that, and
        // correct the robot's heading.  It's not smart enough to oversteer to make sure we're on the exact
        // same plain, but it's good enough for our use case since people shouldn't hit us in auton
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // leftDrivesetMode(DcMotor.RunMode.RUN_TO_POSITION);
        // rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION;
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double LeftPower = 0;
        double RightPower = 0;
        int Direction = -direction;
        double diff = -getError(targetAngle);
        // Play with this value for different sensitivities depending on different speeds
        double PVal = 15/targetSpeed;
        if(Direction == -1){
            //if we're traveling backwards, we want to add the difference to the opposite as we
            // would if we traveled forward
            //We're getting the targetSpeed, and adding the (difference/PVal)
            //The PVal is decided by dividing 15 (which is an arbitrary value determined by robot)
            //  by the target speed.  It was played around with, and decided on after testing.
            // By including a second method of tuning our speed changes, we can have a more,
            // or less, sensitive proportional drive depending not just on the error of our heading,
            // but depending on our target speed as well.  This means when we're traveling fast,
            // we change our values more, because it's actually a smaller percentage of it's overall
            // speed.  In contrast, while driving slowly, we make smaller speed changes.
            LeftPower = Direction*(targetSpeed-diff/PVal);
            RightPower = Direction*(targetSpeed-diff/PVal);
        }else{
            //same as above, but opposite
            LeftPower = Direction*(targetSpeed+diff/PVal);
            RightPower = Direction*(targetSpeed+diff/PVal);
        }
        //Make sure the powers are between 1 and -1.  This doesn't do much, other than ensure
        // stability of the code, and making sure it doesn't crash for a weird reason
        leftDrive.setPower(Range.clip(LeftPower, -1, 1));
        rightDrive.setPower(Range.clip(RightPower, -1, 1));
    }

    public double getError(double targetAngle) {
        //This function compares the current heading to the target heading, and returns the error
        double robotError;
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    

    
    public void SimpleEncoderTurn(double speed, double angle, double timeout) {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Basically, run with encoder for a set distance that we set above 
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double startTime = runtime.seconds();
        
        if (angle>0){
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            //Which direction to turn? These will set it so that it turns left or right depending on the angle.
        }
        if (angle<0){
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
        }
        

        
        rightDrive.setPower(0.1); //Basically, give the motors a small start so that the while loop can activate
        leftDrive.setPower(0.1); //Might get rid of this...
        imu.resetYaw();
        double error = 1;
        
        
        while ((!(getHeading()>(angle-0.4) && getHeading()<(angle+0.4))&&(error!=0)) && ((startTime+timeout)<runTime.seconds())){
            
            error = 1-(getHeading()/angle); 
            int cof = 1000;
            //1 - the normalized value so that starting distance is one, ending distance = 0. ^^ MATHHH
            //double targetVelocity = (Math.pow(1.5,(error-1)*9))*speed*3000; //Confused? Open desmos and look from 0 to 1,
            //get rid of * 1000 and speed and stuff. Just a velocity curve
            if (error<0.004){
                error=0;
                cof = 3000;
            }
            double targetVelocity = ((-1/Math.pow(2, ((3)*error)))+1)*speed*cof;
            //double targetVelocity = ((0.5*(Math.sin(Math.PI*error-(0.5*Math.PI))))+0.5)*speed*1000; --NEEDS TO BE TESTED - REMOVE COMMENT ONCE TESTED
            //FOR 0.5 SPEED -- 1.4 SCALING BASE -- SECOND EQUATION CURRENT -- TO SWITCH EQUATION, UNCOMMENT ONE AND COMMENT ANOTHER
            ((DcMotorEx)(rightDrive)).setVelocity(targetVelocity);
            ((DcMotorEx)(leftDrive)).setVelocity(targetVelocity); 
            //Type cast to the version of motor that can be used with PID velocity control while in RunToPosition mode.
            
            telemetry.addData("Running Angle:", angle);
            //Target heading
            telemetry.addData("targetPos",rightDrive.getTargetPosition()); //Target for motors
            telemetry.addData("currentPos",rightDrive.getCurrentPosition()); //Current motor position
            telemetry.addData("currentVelocity", ((DcMotorEx) rightDrive).getVelocity()); //Typecast to motor premium, in essence, to read velocity
            telemetry.addData("targetVelocity", targetVelocity); //The curve value -- Make sure we are following the curve
            telemetry.addData("getHeading", getHeading());
            //Current heading
            telemetry.addData("error", error); //Will return progress of turn in essence,
            //1 is the start, 0 is the end, imagine right to left
            telemetry.update();
            
            

        }
        rightDrive.setPower(0.0);
        leftDrive.setPower(0.0);
        // sleep(4000); //Debug sleep
        

    }

    //Begin April Tag Functions: 
    public void changeDesiredTag(int tag){
        DESIRED_TAG_ID = tag;
    }
    public void setDecimation(int dec){
        aprilTag.setDecimation(dec);
    }
    private void initAprilTag() {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                BuiltinCameraDirection.BACK, aprilTag);
        }
    }   // end method initAprilTag()
    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }
        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }
    public void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;
        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max >1.0) {
            leftPower /= max;
            rightPower /= max;
        }
        // Send powers to the wheels.
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void aprilTagDetector(){
        if (opModeIsActive()){
            boolean targetFound     = false;    // Set to true when an AprilTag target is detected
            double  drive           = 0;        // Desired forward power/speed (-1 to +1) +ve is forward
            double  turn            = 0;        // Desired turning power/speed (-1 to +1) +ve is CounterClockwise
            
        }
        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
            //first: Rotate around until you locate desired tag
            // {
            // }
            // Tell the driver what we see, and what to do.
            if (targetFound) {
                // telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);

                
                
                if (desiredTag.ftcPose.bearing != 0 && desiredTag.ftcPose.yaw != 0){
                    
                    //set temprorary variables for yaw, range bearing at that moment
                    robBearing = desiredTag.ftcPose.bearing;
                    robRange = desiredTag.ftcPose.bearing;
                    robYaw = desiredTag.ftcPose.yaw;
                    boolean posRotate = true;
                    //move robot to be aligned with the yaw
                    //HardCode.rotateRobot(robYaw);
                    SimpleEncoderTurn(0.5, robYaw, 5);
                    //rotate to the left or right depending on the value of the yaw
                    
                    //move to the left using the cos code
                    double needDistance = Math.cos(90 - robBearing) * robRange;
                    //HardCode.setDesiredDistance(needDistance);
                    //rotate right by 90 degrees the other way
                    telemetry.addData("Yaw", "Turn 90 degrees opposite of the first direction to look at tag");
                    if(robYaw < 0)
                    {
                        SimpleEncoderTurn(0.5,90,5);
                        telemetry.addData("Yaw","rotate 90 degress right");
                    }
                    else if(robYaw > 0)
                    {
                        SimpleEncoderTurn(0.5,-90,5);
                        telemetry.addData("Yaw","rotate 90 degrees left");
                    }

                    SimpleEncoderDrive(0.5,(int)needDistance, forward, 10);
                }

    }

    }
    

    


    }
}
