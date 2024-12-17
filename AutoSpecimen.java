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


    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 150 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 130 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 60 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 60 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 20 * ARM_TICKS_PER_DEGREE;
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
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD //This decides axis relative to our Robot....
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
            SimpleEncoderDrive(0.5,48.0,forward);
        }

        // encoderDriveSmooth(5.0,24.0,-1,0,5); //move backward 24 inches

        // encoderDriveSmooth(25.0,24,1,130,5); //heading test

        












        //We will build the rest of this once the other functions are made...
    }

    public double getHeading(){
        //returns the Yaw -- Makes it easier so I can just refrence 
        //heading as just that instead of having to think of Yaw, Pitch, Roll etc...
        robotOrientation = imu.getRobotYawPitchRollAngles();
        Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        return Yaw;
    }


    //This section needs work! //NOTE -- THIS CODE IS LEGACY
    //Note: Someone will have to clean up these nasty telemetry statements at some point and comments
    public void encoderDriveSmooth(double speed, double Inches, int direction, double heading, double timeout) { 
        /**Speed is how fast we want our robot to go to a set target. NOTE: THIS SHOULD BE A VALUE NORMALIZED 
         * Inches is the distance, in Inches. Positive.
         * Direction should be filled with the forward constant -- Use forward or reverse. 
         * Heading is the degrees we should be going to. For staying straight, use current heading.
         * Timeout will be used at a later date once it is worked out.
        */
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

    public void SimpleEncoderDrive(double speed, double Inches, double direction){

        if (opModeIsActive() ) {
            
            double targetSpeed = 0;

        
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setTargetPosition((int)(Inches*CountsPerInch));
            rightDrive.setTargetPosition((int)(Inches*CountsPerInch));
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Basically, run with encoder for a set distance. 
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            if (Inches > 1){
                rightDrive.setPower(0.1);
                leftDrive.setPower(0.1);
                sleep(100);
            }
            else {
                telemetry.addLine("Inches invalid, must be greater than 1!");
            }


            while (leftDrive.isBusy() || leftDrive.isBusy()){

                double error = 1-((double)rightDrive.getCurrentPosition())/((double)rightDrive.getTargetPosition());
                double targetVelocity = ((-1/Math.pow(3, (9*error)))+1)*speed*1000; //Confused? Open desmos and look from 0 to 1, get rid of * 1000 and speed
                ((DcMotorEx)(rightDrive)).setVelocity(targetVelocity);
                ((DcMotorEx)(leftDrive)).setVelocity(targetVelocity);
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
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            //leftDrive.setPower(0);
            //rightDrive.setPower(0);
            telemetry.addData("Ran Distance:", Inches);
            //telemetry.addData("LongSpeed",(int)temp);
            telemetry.update();
            sleep(4000);

        }

    }



    
    public void gyroDrive(double targetAngle, double targetSpeed, int direction) {
        
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


    }
