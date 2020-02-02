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
@Autonomous(name = "Skystone Detector OpMode", group="DogeCV")

public class DogeTest extends LinearOpMode {
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
    public void runOpMode() {

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
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
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

            if(skyStoneDetector.getScreenPosition().x > 100){
                int final_back1 = (int) Math.round(2 * ticks_per_inch);

                reset_motor();
                fl.setTargetPosition(-final_back1);
                bl.setTargetPosition(-final_back1);
                fr.setTargetPosition(+final_back1);
                br.setTargetPosition(+final_back1);
                powerBusy(0.5);

                sleep(500);
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