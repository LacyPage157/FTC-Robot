package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.external.android;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name="AutoSpecimen", group="Robot")
public class AutoSpecimen extends LinearOpMode
{



}

public void rotateRobot(double degrees)
{
    targetRotations=(434/96)*(degrees/360);
    rightDrive.setDirection(DcMotor.Direction.FORWARD);
    runOpMode();
}

public void moveRobot(double x, double yaw) 
{
        // Calculate left and right wheel powers.
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;
}
        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max >1.0) {
            leftPower /= max;
            rightPower /= max;
        }
