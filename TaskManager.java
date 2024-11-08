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


public static class TaskManager {
    public int phase = 0;


    public static main(int phase){

        switch(phase){
            case 0:
                ColorLocator.runOpMode();
                telemetry.addData("Status","ColorLocator.java");
                break;

            case 1:
                AprilTagRed.runOpMode(); //NOTE: AprilTagRed is the same as AprilTagBlue, as such we will use RED as a universal.
                
                telemetry.addData("Status","AprilTagRed.java");

                break;

            case 2:
                Orientate.setDesiredTag(-1); //FOR NOW -1 -- Later will implement a more rigid system for what we are trying to look for.
                Orientate.runOpMode();
                telemetry.addData("Status","Orientate.java");
                break;
            case 3:
                HardCode.runOpMode();
                telemetry.addData("Status","HardCode.java");
                break;

///Probably reset here. 
//Run hard code after 

            default:
                telemetry.addData("Status","Error:DefaultCaseAtPhaseUndefined");
                break;

        }
        telemetry.update()
        }







    }




