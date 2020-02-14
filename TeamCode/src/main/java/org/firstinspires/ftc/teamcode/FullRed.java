package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Locale;

/*
 * Thanks to EasyOpenCV for the great API (and most of the example)
 *
 * Original Work Copright(c) 2019 OpenFTC Team
 * Derived Work Copyright(c) 2019 DogeDevs
 */
@Autonomous(name = "FullRed", group="DogeCV")

public class FullRed extends LinearOpMode {
    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;

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

    @Override
    public void runOpMode() throws InterruptedException{

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

        Servo test = hardwareMap.get(Servo.class, "test");

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

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);

        /*
         * Wait for the user to press start on the Driver Station
         */

        //test.setPosition(1);

        waitForStart();

        if (opModeIsActive()) {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
            telemetry.addData("Stone Position Y", skyStoneDetector.getScreenPosition().y);
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();
            boolean bkah = true;
            int count = 0;
            int foundationMovement = 0;
            int oppositeSide = 0; //used for the first case since the robot move s too close to the wall
            //turn towards the skystone
            while (bkah) {
                telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
                if (skyStoneDetector.getScreenPosition().x > 200) {
                    backwards(12, .3);
                    int final_back1 = (int) Math.round(4.4 * ticks_per_inch);

                    reset_motor();
                    fl.setTargetPosition(-final_back1);
                    bl.setTargetPosition(-final_back1);
                    fr.setTargetPosition(+final_back1);
                    br.setTargetPosition(+final_back1);
                    powerBusy(0.5);

                    sleep(500);
                    bkah = false;
                    count = 1;
                    foundationMovement = 2;
                    oppositeSide = -3;
                }
                else if(skyStoneDetector.getScreenPosition().x < 100){
                    backwards(10, .3);
                    int final_back1 = (int) Math.round(3 * ticks_per_inch);

                    reset_motor();
                    fl.setTargetPosition(+final_back1);
                    bl.setTargetPosition(+final_back1);
                    fr.setTargetPosition(-final_back1);
                    br.setTargetPosition(-final_back1);
                    powerBusy(0.5);

                    sleep(500);
                    bkah = false;
                    count = 2;
                }
                else{
                    backwards(10 , .3);
                    sleep(500);
                    bkah = false;
                    foundationMovement = 2;
                }
                telemetry.update();
            }



            //move forward and intake
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



            Thread.sleep(1200);
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);

            intakeLeft.setPower(0); //stop the intake wheels
            intakeRight.setPower(0);

            intakeLeft.setPower(-0.7); //make the intake wheels move
            intakeRight.setPower(0.7);

            Thread.sleep(500);

            intakeLeft.setPower(0); //stop the intake wheels
            intakeRight.setPower(0);

            sleep(100);

            fl.setPower(.5);
            fr.setPower(.5);
            bl.setPower(.5);
            br.setPower(.5);
            //move back to orginal position
            Thread.sleep(1300);

            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);


            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(1000);
            if(count == 1){
                int final_back1 = (int) Math.round(4.4 * ticks_per_inch);

                reset_motor();
                fl.setTargetPosition(+final_back1);
                bl.setTargetPosition(+final_back1);
                fr.setTargetPosition(-final_back1);
                br.setTargetPosition(-final_back1);
                powerBusy(0.5);


            }else if(count == 2){
                int final_back1 = (int) Math.round(3 * ticks_per_inch);

                reset_motor();
                fl.setTargetPosition(-final_back1);
                bl.setTargetPosition(-final_back1);
                fr.setTargetPosition(+final_back1);
                br.setTargetPosition(+final_back1);
                powerBusy(0.5);
            }
            sleep(500);
            //get back to 0 degrees
           /* while (opModeIsActive() && !isStopRequested()) {
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
            }*/
            //move forward so you don't hit the robot
            backwards(8,.5);

            //to turn right to move to the foundation
            int final_back = (int) Math.round(18 * ticks_per_inch);

            reset_motor();
            fl.setTargetPosition(-final_back);
            bl.setTargetPosition(-final_back);
            fr.setTargetPosition(+final_back);
            br.setTargetPosition(+final_back);
            powerBusy(0.5);

            sleep(200);


            while (opModeIsActive() && !isStopRequested()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading:", angles.firstAngle);
                sleep(200);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


                if (angles.firstAngle < -89) {
                    //to turn left
                    int final_back2 = (int) Math.round(1 * ticks_per_inch);

                    reset_motor();
                    fl.setTargetPosition(+final_back2);
                    bl.setTargetPosition(+final_back2);
                    fr.setTargetPosition(-final_back2);
                    br.setTargetPosition(-final_back2);
                    powerBusy(0.5);

                } else if (angles.firstAngle > -85){
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

            //go to foundation
            backwards(75 + oppositeSide,.7);

            sleep(300);

            //turn left towards the foundation
            final_back = (int) Math.round(16 * ticks_per_inch);

            reset_motor();
            fl.setTargetPosition(+final_back);
            bl.setTargetPosition(+final_back);
            fr.setTargetPosition(-final_back);
            br.setTargetPosition(-final_back);
            powerBusy(0.5);

            sleep(200);
            //get back to 0 degrees
            //autocorrect to align with foundation
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
            //move towards the foundation
            backwards(15,.2);

            //clamp on
            leftServo.setPosition(-0.5); //clamp on
            rightServo.setPosition(0.4);
            sleep(1500);

            //move to building site
            forward(25 + foundationMovement,.4);
            sleep(200);

            //turn left to move foundation into depot
            int final_back2 = (int) Math.round(60 * ticks_per_inch);

            reset_motor();
            fl.setTargetPosition(-final_back2);
            bl.setTargetPosition(-final_back2);
            fr.setTargetPosition(+final_back2);
            br.setTargetPosition(+final_back2);
            powerBusy(0.5);

            //let go of the foundation
            leftServo.setPosition(1);
            rightServo.setPosition(0);
            sleep(300);

            backwards(4, 0.5); //bang into the wall
            sleep(500);

            forward(3, 0.4); //move a little backwards to relieve friction before strafing
            //sleep(200);

            straferight(12, 0.5); //move to the side of the foundation
           // sleep(500);

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
            Thread.sleep(600);
            sleep(600);
            intakeLeft.setPower(0);
            intakeRight.setPower(0);

            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            forward(13, 0.7); //park
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
    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*ticks_per_inch));
        //
        bl.setTargetPosition(bl.getCurrentPosition() + move);
        fl.setTargetPosition(fl.getCurrentPosition() + move);
        br.setTargetPosition(br.getCurrentPosition() + move);
        fr.setTargetPosition(fr.getCurrentPosition() + move);
        //
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        fl.setPower(speed);
        bl.setPower(speed);
        fr.setPower(speed);
        br.setPower(speed);
        //
        while (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy()){

                fr.setPower(0);
                fl.setPower(0);
                br.setPower(0);
                bl.setPower(0);
                return;

        }
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        return;
    }
    //
    /*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
        //</editor-fold>
        //
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void waitForStartify(){
        waitForStart();
    }
    //
    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input){
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        fl.setPower(input);
        bl.setPower(input);
        fr.setPower(-input);
        br.setPower(-input);
    }
    //
}