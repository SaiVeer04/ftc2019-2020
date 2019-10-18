package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp1")

public class TeleOp extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    DcMotor FR = null; //declaration of motors
    DcMotor FL = null;
    DcMotor BR = null;
    DcMotor BL = null;

    DcMotor intakeLeft;
    DcMotor intakeRight;

    DcMotor string, rotate;
    public static final double INCREMENT = .1;
    public void runOpMode() throws InterruptedException {



        Servo test = hardwareMap.get(Servo.class, "test");
        Servo paddle = hardwareMap.get(Servo.class, "paddle");


        FR = hardwareMap.get(DcMotor.class, "FR"); //initilization
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");

        string = hardwareMap.get(DcMotor.class, "string");
        rotate = hardwareMap.get(DcMotor.class, "rotate");


        FL.setDirection(DcMotorSimple.Direction.REVERSE); //set left side motors to opposite so that the robot moves
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       /*string.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //used fo encoders
        string.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/


        waitForStart(); //when the driver clicks play


        if (opModeIsActive()) {
            while (opModeIsActive()) {

                double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = gamepad1.right_stick_x;
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;

                FL.setPower(v1);
                FR.setPower(v2);
                BL.setPower(v3);
                BR.setPower(v4);

                intakeLeft.setPower(gamepad1.left_trigger * 2);
                intakeRight.setPower(gamepad1.left_trigger * 2);

                intakeLeft.setPower(-gamepad1.right_trigger * 2);
                intakeRight.setPower(-gamepad1.right_trigger * 2);

                if (gamepad1.x) {
                    test.setPosition(0);
                }

                if (gamepad1.b) {
                    test.setPosition(1);
                }

                if(gamepad1.a){
                    string.setPower(0.5);
                }

                else if(gamepad1.y){
                    string.setPower(-0.5);
                }
                else if (gamepad1.a == false || gamepad1.y == false){
                    string.setPower(0);
                }

                //motor rotation code
                if(gamepad1.dpad_left){
                    rotate.setPower(0.25);
                }

                else if(gamepad1.dpad_right){
                    rotate.setPower(-0.25);
                }
                else if (gamepad1.dpad_left == false || gamepad1.dpad_right == false){
                    rotate.setPower(0);
                }
                //string.setPower(boolToInt(gamepad1.y, 0)*2);
               // string.setPower(-(boolToInt(gamepad1.a, 0)));

               // rotate.setPower(boolToInt(gamepad1.dpad_right, .6));
               // rotate.setPower(-(boolToInt(gamepad1.dpad_left, .1)));
               /* if(gamepad1.dpad_left) {
                    rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rotate.setTargetPosition(100);
                    rotate.setPower(.2);
                }
                if(gamepad1.dpad_right) {
                    rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rotate.setTargetPosition(-100);
                    rotate.setPower(.2);
                }*/


               // rotate.setPower(gamepad1.right_trigger);
               // rotate.setPower(-gamepad1.left_trigger);

              /*if (gamepad1.dpad_left)
                  rotate.setPower(1);
              if (gamepad1.dpad_right)
                  rotate.setPower(-1);*/



                if (gamepad1.left_bumper) //decide later
                {
                    paddle.setPosition(0);
                }
                if (gamepad1.right_bumper)
                {
                    paddle.setPosition(1);
                }





                /*while (gamepad1.b) {
                    rotate.setPower(-0.4);
                    if (!gamepad1.b) {
                        rotate.setPower(0);
                    }*/
            }


        }


    }

    double boolToInt(boolean b, double limit) {
        if (b)
            return 1 - limit;
        return 0;
    }

    void movement(int encoders, DcMotor motor) {
        motor.setTargetPosition(encoders); //sets the distance of the motor
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //tells the motor to go the distance of the encoder
        motor.setPower(0.3); //speed of the motor

        while (opModeIsActive() && //use this method to prevent skipping of encoders
                motor.isBusy()) {

        }

        // Stop all motion;
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn off RUN_TO_POSITION
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}