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
    StoneAlignTest detector = new StoneAlignTest();

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
        detector.VUFORIA_KEY = "AVF+IPr/////AAABmSIHgAGLI0ODn254a6Sw3UQK0VqDJcWazHjdrhyfBcJdjHXFe3pv6C0EYG8QGSLhOfCTPGxj3GgfzXF/ndSARshwvj7P3SrpPLgvKVZyl5tPjFYSCIU6r3CzQTmFXGut7tgCZjTS59auWpsAZJSLeO76pI2oqQ5aga+MMDlaQ6i2IM3TbaqrcamwoPfElmTc/kb6qMqibv98MGhAflk0Rv1fHEoTjmBw6WzMI5pWn5QEPtjwW2JaS5JsLZu0jQWu9qn6Wz35u9yLrs8rA8ChOIvQemWFUuTzlteADKNPnogFOWZQv4iur/22GphGP+Cu/65iAV6r+RkBnQ3oiRspOi3J4QliYBnbrSokwkBHiyhW";
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), DogeCV.CameraMode.WEBCAM, false, webcamName);

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

        detector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100); // Create new filter
        detector.useDefaults(); // Use default settings
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // Uncomment if using PERFECT_AREA scoring

        detector.enable();



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


                        if(detector.getXPosition() == 320.0){

                            telemetry.addLine("Aligned");
                        }
                    }
                }
                telemetry.addData("Is Found: ", detector.isFound());
                telemetry.addData("X Pos: ",detector.getXPosition());
                telemetry.addData("Y Pos: ",detector.getYPosition());
                telemetry.update();


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