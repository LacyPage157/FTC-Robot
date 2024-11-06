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
    public static main(){

        switch(phase){
            case 0:
                HardCode.runOpMode();
                telemetry.addData("Status","RunningHardCode.java");
                break;

            case 1:
                ColorLocator.runOpMode();
                telemetry.addData("Status","RunningHardCode.java")

                break;

            case 2:

                break;

            default:
                telemetry.addData("Status","Error:DefaultCaseAtPhaseUndefined");
                break;

        }
        telemetry.update()
        }







    }




