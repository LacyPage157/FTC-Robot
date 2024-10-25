package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOpCode", group="Robot")
public class CoolCode extends LinearOpMode {

    // public DcMotor  le"leftMotor"ive   = null; //the left drivetrain motor
    // public DcMotor  r"rightMotor"Drive  = null; //the right drivetrain motor
    // public DcMotor  "armMotor"otor    = null; //the arm motor
    // public CRServo  intake      = null; //the active intake servo
    // public Servo    wrist       = null; //the wrist servo

    // final double ARM_TICKS_PER_DEGREE =
    //         28 // number of encoder ticks per rotation of the bare motor
    //                 * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
    //                 * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
    //                 * 1/360.0; // we want ticks per degree, not per rotation




    public void runOpMode() {

        if(gamepad1.y) {
            telemetry.addData("Pos","y is being pressed");
        }
        
        telemetry.addData("pos","Everyframe");
        telemetry.update();
        
        waitForStart();
        
        

    }


    
}
