package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "FullBlue")
public class FullBlue extends LinearOpMode{

    float hsvValues[] = {0F, 0F, 0F};

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
    // we use customary because we cool like that
    //all holding values
    static final double wheel_diameter = 4.0;
    static final double gear_ratio = 1.0;
    //for encoders
    static final double ticks = 537.6;
    static final double ticks_per_inch = (ticks * gear_ratio)/(wheel_diameter * Math.PI);

    public DcMotor bl;
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor br;

    public ColorSensor colorSensor;

    public DcMotor intakeLeft;
    //intakeright
    public DcMotor intakeRight;

    public DistanceSensor sensorRange;
    public DistanceSensor sensorRange1;

    public Servo leftServo;

    public Servo rightServo;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;



    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        bl = hardwareMap.dcMotor.get("BL");
        fl = hardwareMap.dcMotor.get("FL");
        fr = hardwareMap.dcMotor.get("FR");
        br = hardwareMap.dcMotor.get("BR");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        leftServo = hardwareMap.get(Servo.class, "leftServo");

        rightServo = hardwareMap.get(Servo.class, "rightServo");


        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        sensorRange1 = hardwareMap.get(DistanceSensor.class, "sensor_range1");
        Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor)sensorRange1;

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        //required to tell using encoder
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double movement = 0;

        waitForStart();

        if (opModeIsActive()) {

            backwards(20, .5);
            sleep(100);


            for (int i = 0; i <= 12; i++) {
                sleep(500);
                Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                        (int) (colorSensor.green() * SCALE_FACTOR),
                        (int) (colorSensor.blue() * SCALE_FACTOR),
                        hsvValues);
                sleep(500);

                if (hsvValues[0] > 80) { //check the hue value
                    //strafeleft(8, .5);

                    sleep(500);

                    fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    fl.setPower(-.5);
                    fr.setPower(-.5);
                    bl.setPower(-.5);
                    br.setPower(-.5);

                    intakeLeft.setPower(-0.7); //make the intake wheels move
                    intakeRight.setPower(0.7);

                    Thread.sleep(400);
                    fl.setPower(0);
                    bl.setPower(0);
                    fr.setPower(0);
                    br.setPower(0);

                    intakeLeft.setPower(0); //stop the intake wheels
                    intakeRight.setPower(0);

                    sleep(100);

                    intakeLeft.setPower(-0.7); //make the intake wheels move
                    intakeRight.setPower(0.7);

                    fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    fl.setPower(-.5);
                    fr.setPower(-.5);
                    bl.setPower(-.5);
                    br.setPower(-.5);

                    Thread.sleep(200);

                    intakeLeft.setPower(0); //stop intake wheels
                    intakeRight.setPower(0);

                    fl.setPower(0);
                    bl.setPower(0);
                    fr.setPower(0);
                    br.setPower(0);

                    intakeLeft.setPower(-0.7); //make the intake wheels move
                    intakeRight.setPower(0.7);
                    Thread.sleep(200);
                    intakeLeft.setPower(0); //stop intake wheels
                    intakeRight.setPower(0);

                    sleep(500);


                    fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    forward(24, 0.7); //moves the robot back
                    sleep(200);

                    //to turn right
                    int final_back = (int) Math.round(16 * ticks_per_inch);

                    reset_motor();
                    fl.setTargetPosition(+final_back);
                    bl.setTargetPosition(+final_back);
                    fr.setTargetPosition(-final_back);
                    br.setTargetPosition(-final_back);
                    powerBusy(0.5);

                    sleep(200);



                   while (opModeIsActive() && !isStopRequested()) {
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        telemetry.addData("Heading:", angles.firstAngle);
                        sleep(200);
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


                        if (angles.firstAngle < 88) {
                            //to turn left
                            int final_back1 = (int) Math.round(1 * ticks_per_inch);

                            reset_motor();
                            fl.setTargetPosition(+final_back1);
                            bl.setTargetPosition(+final_back1);
                            fr.setTargetPosition(-final_back1);
                            br.setTargetPosition(-final_back1);
                            powerBusy(0.5);

                        } else if (angles.firstAngle > 92){
                            //to turn right
                            int final_back1 = (int) Math.round(1 * ticks_per_inch);

                            reset_motor();
                            fl.setTargetPosition(-final_back1);
                            bl.setTargetPosition(-final_back1);
                            fr.setTargetPosition(+final_back1);
                            br.setTargetPosition(+final_back1);
                            powerBusy(0.5);
                        }
                        else {
                            break;
                        }
                        telemetry.update();
                    }





                    backwards(70 + movement, 0.5); //move to the foundation side (68 was original value)

                    //to turn left
                    int final_back1 = (int) Math.round(18 * ticks_per_inch);

                    reset_motor();
                    fl.setTargetPosition(-final_back1);
                    bl.setTargetPosition(-final_back1);
                    fr.setTargetPosition(+final_back1);
                    br.setTargetPosition(+final_back1);
                    powerBusy(0.5);

                    sleep(500);


                    while (opModeIsActive() && !isStopRequested()) {
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        telemetry.addData("Heading:", angles.firstAngle);
                        sleep(200);
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


                        if (angles.firstAngle < -5) {
                            //to turn left
                            int final_back2 = (int) Math.round(1 * ticks_per_inch);

                            reset_motor();
                            fl.setTargetPosition(+final_back2);
                            bl.setTargetPosition(+final_back2);
                            fr.setTargetPosition(-final_back2);
                            br.setTargetPosition(-final_back2);
                            powerBusy(0.5);

                        } else if (angles.firstAngle > 4){
                            //to turn right
                            int final_back2 = (int) Math.round(1 * ticks_per_inch);

                            reset_motor();
                            fl.setTargetPosition(-final_back2);
                            bl.setTargetPosition(-final_back2);
                            fr.setTargetPosition(+final_back2);
                            br.setTargetPosition(+final_back2);
                            powerBusy(0.5);
                        }
                        else {
                            break;
                        }
                        telemetry.update();
                    }

                    backwards(15, 0.3); //move forward to the foundation



                    leftServo.setPosition(-0.5); //clamp on
                    rightServo.setPosition(0.4);
                    sleep(2500);


                    forward(30, 0.7); //move backward
                    sleep(200);

                    //turn left to move foundation into depot
                    int final_back2 = (int) Math.round(52 * ticks_per_inch);

                    reset_motor();
                    fl.setTargetPosition(+final_back2);
                    bl.setTargetPosition(+final_back2);
                    fr.setTargetPosition(-final_back2);
                    br.setTargetPosition(-final_back2);
                    powerBusy(0.5);



                    leftServo.setPosition(1); //let go
                    rightServo.setPosition(0);
                    sleep(300);

                    backwards(4, 0.5); //bang into the wall
                    sleep(500);

                    forward(2, 0.4); //move a little backwards to relieve friction before strafing
                    sleep(200);

                    strafeleft(12, 0.5); //move to the side of the foundation
                    sleep(500);

                    backwards(7, 0.5);
                    fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    fl.setPower(.5);
                    fr.setPower(.5);
                    bl.setPower(.5);
                    br.setPower(.5);

                    intakeLeft.setPower(0.7); //outtake
                    intakeRight.setPower(-0.7);
                    sleep(600);
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);

                    fl.setPower(0);
                    fr.setPower(0);
                    bl.setPower(0);
                    br.setPower(0);

                    forward(17, 0.5); //park





                    break;

                } else {
                    movement += 7;

                    //forward(2,.3);



                    strafeleft(10, .8); //move to the next block
                    backwards(2, 0.5);

                    while (opModeIsActive() && !isStopRequested()) {
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        telemetry.addData("Heading:", angles.firstAngle);
                        sleep(200);
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


                        if (angles.firstAngle < -2) {
                            //to turn left
                            int final_back2 = (int) Math.round(1 * ticks_per_inch);

                            reset_motor();
                            fl.setTargetPosition(+final_back2);
                            bl.setTargetPosition(+final_back2);
                            fr.setTargetPosition(-final_back2);
                            br.setTargetPosition(-final_back2);
                            powerBusy(0.5);

                        } else if (angles.firstAngle > 2){
                            //to turn right
                            int final_back2 = (int) Math.round(1 * ticks_per_inch);

                            reset_motor();
                            fl.setTargetPosition(-final_back2);
                            bl.setTargetPosition(-final_back2);
                            fr.setTargetPosition(+final_back2);
                            br.setTargetPosition(+final_back2);
                            powerBusy(0.5);
                        }
                        else {
                            break;
                        }
                        telemetry.update();
                    }
                }
            }


        }
    }
    public void reset_motor(){
        //resets encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //this one lets each movement run to the full extent with out overriding
    public void powerBusy(double power) {
        //lets program run fully
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
        while ((fl.isBusy() && fr.isBusy())&&(bl.isBusy() && br.isBusy())){}
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    public void forward(double front,double power){
        int final_front = (int)Math.round(front*ticks_per_inch);

        reset_motor();
        fl.setTargetPosition(final_front);
        bl.setTargetPosition(final_front);
        fr.setTargetPosition(final_front);
        br.setTargetPosition(final_front);
        powerBusy(power);

    }
    public void backwards(double back,double power){

        int final_back = (int)Math.round(back*ticks_per_inch);

        reset_motor();
        fl.setTargetPosition(-final_back);
        bl.setTargetPosition(-final_back);
        fr.setTargetPosition(-final_back);
        br.setTargetPosition(-final_back);
        powerBusy(power);

    }
    public void strafeleft(double back,double power){

        int final_back = (int)Math.round(back*ticks_per_inch);

        reset_motor();
        fl.setTargetPosition(-final_back);
        bl.setTargetPosition(final_back);
        fr.setTargetPosition(final_back);
        br.setTargetPosition(-final_back);
        powerBusy(power);

    }
    public void straferight(double back,double power){

        int final_back = (int)Math.round(back*ticks_per_inch);

        reset_motor();
        fl.setTargetPosition(final_back);
        bl.setTargetPosition(-final_back);
        fr.setTargetPosition(-final_back);
        br.setTargetPosition(final_back);
        powerBusy(power);

    }

    public void turn(double degrees,String l_or_r,double power){
        if(l_or_r.equals("l")){
            int leftvalue = (int)((degrees/360)* 1120);
            reset_motor();
            fl.setTargetPosition(-leftvalue);
            bl.setTargetPosition(-leftvalue);
            fr.setTargetPosition(leftvalue);
            br.setTargetPosition(leftvalue);
            powerBusy(power);

        }else if(l_or_r.equals("r")){
            int leftvalue = (int)((degrees/360)* 1120);
            reset_motor();
            fl.setTargetPosition(leftvalue);
            bl.setTargetPosition(leftvalue);
            fr.setTargetPosition(-leftvalue);
            br.setTargetPosition(-leftvalue);
            powerBusy(power);

        }

    }


    }
