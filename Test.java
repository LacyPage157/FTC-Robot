package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name="Motor Position Test", group="Robot")
//@Disabled
public class Test extends LinearOpMode {

    public DcMotor  leftDrive   = null; //the left drivetrain motor
    public DcMotor  rightDrive  = null;
    public double left;
    public double right;
    public double forward;
    public double rotate;
    public double max;
    public int target = 90;


    final double MOTOR_TICKS_PER_ROTATION =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0; // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox

    @Override
    public void runOpMode(){
        leftDrive  = hardwareMap.get(DcMotor.class, "rightMotor"); //the left drivetrain motor
        rightDrive = hardwareMap.get(DcMotor.class, "leftMotor"); //the right drivetrain motor
        
        
        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Robot Ready.");
        telemetry.addData("MotorTicksPerRotation", MOTOR_TICKS_PER_ROTATION);
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {

            /* Set the drive and turn variables to follow the joysticks on the gamepad.
            the joysticks decrease as you push them up. So reverse the Y axis. */


            
            forward = -gamepad2.left_stick_y;
            rotate  = gamepad2.right_stick_x;

            telemetry.addData("Forward", forward);

            left  = forward + rotate;
            right = forward - rotate;

            /* Normalize the values so neither exceed +/- 1.0 */
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            /* Set the motor power to the variables we've mixed and normalized */
            leftDrive.setPower(left);
            //rightDrive.setPower(right);
            telemetry.addData("TargetPos", target);
            telemetry.addData("currentLeftPos", leftDrive.getCurrentPosition());

            if (target == leftDrive.getCurrentPosition()){
                break;
            }
            telemetry.update();
            
    }
    telemetry.addData("Congrats", "target == leftDrive.getCurrentPostion was true!");
    telemetry.update();
    



}
}