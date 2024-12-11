package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    double CountsPerRev = 28; // number of encoder ticks per rotation of the bare motor
    double GearRatio = 250047.0 / 4913.0; // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
    double WheelDiameterInches = 3.77953;     // For figuring out circumference
    double CountsPerInch = ((CountsPerRev / ((WheelDiameterInches * 3.1415926535))/GearRatio)); 

    //Navigation basic variables
    public double turningSpeed = .4;
    public int forward = 1;
    public int reverse = -1;
    public double left;
    public double right;
    public double rotate;
    public double max;

    double programStartOrientation;
    public double stayOnHeading = 84.17; //Arbitrary number decided more or less...
        

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
        

        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        // Now use these simple methods to extract each angle
        // (Java type double) from the object you just created:
        public double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES); //NOTE: HEADING IS YAW!!!! THEY ARE THE SAME THING!!! 
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

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












        //We will build the rest of this once the other functions are made...
    }

    public double getHeading(){
        //returns the Yaw -- Makes it easier so I can just refrence 
        //heading as just that instead of having to think of Yaw, Pitch, Roll etc...
        return Yaw;
    }


    //This section needs work! 
    public void encoderDriveSmooth(double speed, double Inches, int direction, double heading, double timeout) { 
        /**Speed is how fast we want our robot to go to a set target.  
         * Inches is the distance, in Inches. 
         * Direction should be filled with the forward constant
         * Heading is the degrees we should be going to. For 
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
            //make sure that the encoder on the front left motor (which, stupidly, is the only motor
            //we use for distance in this function) is reset to 0
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //find the amount of encoder ticks to travel based off of the Inches var
            target = leftDrive.getCurrentPosition() + (int) (Inches * CountsPerInch * direction);
            //while the opmode is still running, and we're not at our target yet, and we haven't timed out
            while(opModeIsActive() && notAtTarget && Math.abs(target) - Math.abs(leftDrive.getCurrentPosition()) > 25
                    && (startTime + timeout > runtime.seconds())) {
                //use gyrodrive to set power to the motors.  We have the Heading Var decied earlier,
                // and speed and direction change base off of speed and direciton given by the user
                gyroDrive(Heading, speed, direction);
                
            }
        }
    }

    
    public void gyroDrive(double targetAngle, double targetSpeed, int direction) {
        //For use with other functions, but lets us use the gyro to keep the robot on a certain heading
        // it's proportional, so if for instance, a robot hits us, this will account for that, and
        // correct the robot's heading.  It's not smart enough to oversteer to make sure we're on the exact
        // same plain, but it's good enough for our use case since people shouldn't hit us in auton
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // LeftBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // RightTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            LeftPower = Direction*(targetSpeed+diff/PVal);
            RightPower = Direction*(targetSpeed-diff/PVal);
        }else{
            //same as above, but opposite
            LeftPower = Direction*(targetSpeed-diff/PVal);
            RightPower = Direction*(targetSpeed+diff/PVal);
        }
        //Make sure the powers are between 1 and -1.  This doesn't do much, other than ensure
        // stability of the code, and making sure it doesn't crash for a weird reason
        leftDrive.setPower(Range.clip(LeftPower, -1, 1));
        rightDrive.setPower(Range.clip(RightPower, -1, 1));
    }


    }
