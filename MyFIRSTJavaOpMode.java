package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp

public class MyFIRSTJavaOpMode extends LinearOpMode {
    //private Gyroscope imu;
    private DcMotor motorTest;
    //private DigitalChannel digitalTouch;
    //private ColorSensor sensorColor;
    //private Servo servoTest;


    @Override
    public void runOpMode() {
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColor = hardwareMap.get(ColorSensor.class, "sensorColor");
        //servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        // digitalTouch.setMode(DigitalChannel.Mode.INPUT);r
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        while (opModeIsActive()) {
            
            

            tgtPower = -this.gamepad1.left_stick_y;
            // tgtPower = -this.gamepad1.left_stick_y;
            motorTest.setPower(tgtPower);
            // check to see if we need to move the servo.
            // if(gamepad1.y) {
            //     // move to 0 degrees.
            //     servoTest.setPosition(0);
            // } else if (gamepad1.x || gamepad1.b) {
            //     // move to 90 degrees.
            //     servoTest.setPosition(0.5);
            // } else if (gamepad1.a) {
            //     // move to 180 degrees.
            //     servoTest.setPosition(1);
            // }
            
            // if(tgtPower > 0)
            // {
            //     motorTest.setMotorSpeed(30);
            //     telemetry.addData("Motor speed has been set to 30!");
            // }
            // else if(tgtPower < 0)
            // {
            //     motorTest.setMotorSpeed(-30);
            //     telemetry.addData("Motor speed has been set to -30");
            // }
            // else
            // {
            //     motorTest.setMotorSpeed(0);

            // }
            

            //telemetry.addData("Motor Position", motorTest.getPosition());
            
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Status", "Running");
            
            //sensorColor.enableLed(true);
            // if (digitalTouch.getState() == false) {
            //     // button is pressed.
            //     telemetry.addData("Button", "PRESSED");
            // } else {
            //     // button is not pressed.
            //     telemetry.addData("Button", "NOT PRESSED");
            // }

            telemetry.addData("Status", "Running");
            //telemetry.addData("Status", "Running");

            //telemetry.addData("Best Match:", result.closestSwatch);

            telemetry.update();




        }
    }
}