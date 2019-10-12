package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "hello")

public class test1 extends LinearOpMode {
   GenericDetector detector = new GenericDetector();
    //DcMotor intakeleft;
    //DcMotor intakeright;
    @Override
    public void runOpMode() {
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        //intakeleft = hardwareMap.dcMotor.get("intakeleft");
        //intakeright = hardwareMap.dcMotor.get("intakeright");

       // intakeright.setDirection(DcMotorSimple.Direction.REVERSE);

        //components robot = new components();
        detector.enable();
        waitForStart();
       while (opModeIsActive()) {

           // intakeright.setPower(gamepad1.left_trigger);
           // intakeleft.setPower(gamepad1.left_trigger);

         //   intakeright.setPower(-gamepad1.right_trigger);
          //  intakeleft.setPower(-gamepad1.right_trigger);
           if(detector.isFound()){
               telemetry.addLine("Found");
           }



        }
    }
}
