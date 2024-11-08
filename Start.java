
package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//This code is basically a placeholder code -- We may put a digital sensor if statement in here,
//Bu this is just so that we have a file to run when we set up auto

@Autonomous(name="Start", group="Robot")
public static class Start extends LinearOpMode
{
    
    @Override public void runOpMode()
    {
        TaskManager.main(0);
        Stop();        

    }
}