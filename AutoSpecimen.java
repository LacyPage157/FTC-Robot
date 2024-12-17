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
            SimpleEncoderDrive(0.5,48.0,forward); //Move forward 48 inches at half speed.
        }
        

        //We will build the rest of this once the other functions are made...
    }

    public double getHeading(){
        //returns the Yaw -- Makes it easier so I can just refrence 
        //heading as just that instead of having to think of Yaw, Pitch, Roll etc...
        robotOrientation = imu.getRobotYawPitchRollAngles();
        Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        return Yaw;
    }

    public void SimpleEncoderDrive(double speed, double Inches, double direction){

        if (opModeIsActive() ) {
            
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setTargetPosition((int)(Inches*CountsPerInch*direction));
            rightDrive.setTargetPosition((int)(Inches*CountsPerInch*direction));
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Basically, run with encoder for a set distance that we set above 
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            if (Inches > 1){
                rightDrive.setPower(0.1); //Basically, give the motors a small start so that the while loop can activate
                leftDrive.setPower(0.1);
                sleep(100);
            }
            else {
                telemetry.addLine("Inches invalid, must be greater than 1!");
            }


            while (leftDrive.isBusy() || leftDrive.isBusy()){

                double error = 1-((double)rightDrive.getCurrentPosition())/((double)rightDrive.getTargetPosition());//When I say error, I just mean 
                //1 - the normalized value so that starting distance is one, ending distance = 0. ^^ MATHHH
                double targetVelocity = ((-1/Math.pow(3, ((18*(1.1-speed))*error)))+1)*speed*1000; //Confused? Open desmos and look from 0 to 1, get rid of * 1000 and speed and stuff. Just a velocity curve
                
                ((DcMotorEx)(rightDrive)).setVelocity(targetVelocity);
                ((DcMotorEx)(leftDrive)).setVelocity(targetVelocity); //Type cast to the version of motor that can be used with PID velocity control while in RunToPosition; mode.
                
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
            leftDrive.setPower(0); //Once we are done; with the motors, make sure they are set to zero to eliminate close to 0 power sets.
            rightDrive.setPower(0);
            //leftDrive.setPower(0);
            //rightDrive.setPower(0);
            telemetry.addData("Ran Distance:", Inches);
            //telemetry.addData("LongSpeed",(int)temp);
            telemetry.update();
            sleep(4000); //For testing purposes only

        }

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
