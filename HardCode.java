package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang;

@Autonomous(name="HardCode", group="Robot")
public class HardCode extends LinearOpMode
{
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)

    private double DESIRED_DISTANCE  = null; // Set in inches



    private DcMotor leftDrive   = null;  //  Used to control the left drive wheel
    private DcMotor rightDrive  = null;  //  Used to control the right drive wheel
    private double basePosLeft = null;
    private double basePosRight = null;
    private int rotationsLeft = 0;
    private int rotationsRight = 0;

    private double wheelCircumference = (96*Math.PI)
    private double targetRotations = DESIRED_DISTANCE/wheelCircumference;
    

    @Override public void runOpMode()
    {
        rightDrive = hardwareMap.get(DcMotor.class, "rightMotor");
        leftDrive = hardwareMap.get(DcMotor.class, "leftMotor");
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        basePosLeft = leftDrive.getCurrentPosition();
        basePosRight = rightDrive.getCurrentPosition();
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


        

        

        double drive = Range.clip(1 * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn  = Range.clip(0 * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

            
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        

        while(opModeIsActive()) {
        
        for(int i = (int)targetRotations; rotationsLeft!=i; i++){
            leftDrive.setTargetPosition(360);
            rightDrive.setTargetPosition(360);
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotationsLeft ++;
        }
        leftDrive.setTargetPosition(targetRotations-rotationsLeft);
        leftDrive.setTargetPosition(targetRotations-rotationsRight);
        if ((targetRotations-(int)targetRotations) == leftDrive.getCurrentPosition()){
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
            stop();
        }

            





        }
        //All you Alex & Rishi
        //If possible, use a lot of variables. The easier the modify the code on the fly, the better.
        //
    }

    public void setDesiredDistance(double x){ //X is in inches for consistency
        DESIRED_DISTANCE = x*25.4 //Convert inches to mm
    }

    public void rotateRobot(double degrees){
        targetRotations=(434/96)*(degrees/360);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        runOpMode();





        
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

}

/*
DcMotorController	getController()
Returns the underlying motor controller on which this motor is situated.

int	getCurrentPosition()
Returns the current reading of the encoder for this motor.

DcMotor.RunMode	getMode()
Returns the current run mode for this motor

MotorConfigurationType	getMotorType()
Returns the assigned type for this motor.

int	getPortNumber()
Returns the port number on the underlying motor controller on which this motor is situated.

boolean	getPowerFloat()
Returns whether the motor is currently in a float power level.

int	getTargetPosition()
Returns the current target encoder position for this motor.

DcMotor.ZeroPowerBehavior	getZeroPowerBehavior()
Returns the current behavior of the motor were a power level of zero to be applied.

boolean	isBusy()
Returns true if the motor is currently advancing or retreating to a target position.

void	setMode(DcMotor.RunMode mode)
Sets the current run mode for this motor

void	setMotorType(MotorConfigurationType motorType)
Sets the assigned type of this motor.

void	setPowerFloat()
Deprecated. 
This method is deprecated in favor of direct use of setZeroPowerBehavior() and setPower().

void	setTargetPosition(int position)
Sets the desired encoder target position to which the motor should advance or retreat and then actively hold there at.

void	setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior)
Sets the behavior of the motor when a power level of zero is applied.
*/
