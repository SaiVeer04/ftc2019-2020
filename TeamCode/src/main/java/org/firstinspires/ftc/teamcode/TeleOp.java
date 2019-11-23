package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")

public class TeleOp extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    DcMotor FR = null; //declaration of motors
    DcMotor FL = null;
    DcMotor BR = null;
    DcMotor BL = null;

    DcMotor intakeLeft;
    DcMotor intakeRight;
    WebcamName webcamName;

    DcMotor string, rotate;
    public CRServo foundation;
    public static final double INCREMENT = .1;
    public void runOpMode() throws InterruptedException {
        //init
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");


        Servo test = hardwareMap.get(Servo.class, "test");


        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");

        string = hardwareMap.get(DcMotor.class, "string");
        rotate = hardwareMap.get(DcMotor.class, "rotate");
        foundation = hardwareMap.crservo.get("foundation");


        FL.setDirection(DcMotorSimple.Direction.REVERSE); //set left side motors to opposite so that the robot moves
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        //BR.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        string.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        waitForStart(); //when the driver clicks play


        if (opModeIsActive()) {
            while (opModeIsActive()) {

                /*double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = gamepad1.right_stick_x;
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;

                FL.setPower(v1);
                FR.setPower(v2);
                BL.setPower(v3);
                BR.setPower(v4);*/

                double x = gamepad1.left_stick_x;
                double y = gamepad1.left_stick_y;
                double r = gamepad1.right_stick_x;

                double fl = + x + y + r;
                double fr = - x + y - r;
                double bl = - x - y + r;
                double br = + x - y - r;

                FL.setPower(fl);
                FR.setPower(fr);
                BL.setPower(bl);
                BR.setPower(br);


                intakeLeft.setPower(gamepad1.left_trigger);
                intakeRight.setPower(gamepad1.left_trigger);

                intakeLeft.setPower(-gamepad1.right_trigger);
                intakeRight.setPower(-gamepad1.right_trigger);

                intakeLeft.setPower(gamepad2.left_trigger);
                intakeRight.setPower(gamepad2.left_trigger);

                intakeLeft.setPower(-gamepad2.right_trigger);
                intakeRight.setPower(-gamepad2.right_trigger);

                if (gamepad2.x) {
                    test.setPosition(0);
                }

                if (gamepad2.b) {
                    test.setPosition(0.5);
                }

                if(gamepad2.dpad_down){
                    string.setPower(1);
                }

                else if(gamepad2.dpad_up){
                    string.setPower(-1);
                }
                else if (gamepad2.dpad_up == false || gamepad2.dpad_down == false){
                    string.setPower(0);
                }

                //motor rotation code
                if(gamepad2.dpad_left){
                    rotate.setPower(0.05);
                }

                else if(gamepad2.dpad_right){
                    rotate.setPower(-0.05);
                }
                else if (gamepad2.dpad_left == false || gamepad2.dpad_right == false){
                    rotate.setPower(0);
                }

                if (gamepad2.left_bumper) {
                    foundation.setPower(0.5);
                }

                if (gamepad2.right_bumper) {
                    foundation.setPower(-0.5);
                }

                if (gamepad2.y) {
                    moveForward(-0.05, 3000, 200);
                }


            }
        }


    }

    public void moveForward(double power,int movement,int sleep) throws InterruptedException{
        rotate.setPower(power);
        Thread.sleep(movement);
        rotate.setPower(0);
        sleep(sleep);
    }


        }
