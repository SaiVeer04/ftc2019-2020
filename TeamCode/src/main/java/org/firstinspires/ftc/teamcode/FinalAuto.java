package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Final Auto")
//@Disabled
public class FinalAuto extends LinearOpMode {
    components robot = new components();
    /* Declare OpMode members. */
    // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = .5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AVF+IPr/////AAABmSIHgAGLI0ODn254a6Sw3UQK0VqDJcWazHjdrhyfBcJdjHXFe3pv6C0EYG8QGSLhOfCTPGxj3GgfzXF/ndSARshwvj7P3SrpPLgvKVZyl5tPjFYSCIU6r3CzQTmFXGut7tgCZjTS59auWpsAZJSLeO76pI2oqQ5aga+MMDlaQ6i2IM3TbaqrcamwoPfElmTc/kb6qMqibv98MGhAflk0Rv1fHEoTjmBw6WzMI5pWn5QEPtjwW2JaS5JsLZu0jQWu9qn6Wz35u9yLrs8rA8ChOIvQemWFUuTzlteADKNPnogFOWZQv4iur/22GphGP+Cu/65iAV6r+RkBnQ3oiRspOi3J4QliYBnbrSokwkBHiyhW";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.fl.getCurrentPosition(),
                robot.bl.getCurrentPosition(),
                robot.fr.getCurrentPosition(),
                robot.br.getCurrentPosition());
        telemetry.update();
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        encoderDrive(.2, 0.3, 0.3, -0.3, -0.3, 1.9); //move out
        // sleep(400);
        robot.foundation.setPower(-0.5); //latch
        sleep(2000);
        encoderDrive(.2, -0.3, -0.3, 0.3, 0.3, 1.9); //move back
        sleep(300);
        // robot.foundation.setPower(-1); //unlatch
        // sleep(400);
        // robot.foundation.setPower(0);
        Thread.sleep(1000);


        int p = 10;
        encoderDrive(.3, 5, 5, 5, 5, 3); //move back a little

        while (robot.sensorColor.red() < 40) {
            telemetry.addData("Color sensor", 1);
            encoderDrive(.3, -0.3, 0.3, -0.3, 0.3, p); //strafe right
            sleep(350);
            p += 5;
            //}

            //encoderDrive(.3, -3,-3,-3,-3,3); //move backward a little bit
            // sleep(175);
            // encoderDrive(0.3,2,-2,2,-2, 3);


            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                double imageHeight = recognition.getImageHeight();
                                double objectHeight = recognition.getHeight();
                                double ratio = imageHeight / objectHeight;
                                double angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);

                           /* if (ratio < 5 && ratio > 10) {
                                if (ratio > 10) {
                                    robot.bl.setPower(.2);
                                    robot.fl.setPower(-.2);
                                    robot.fr.setPower(.2);
                                    robot.br.setPower(-.2);
                                } else if (ratio < 5) {
                                    robot.bl.setPower(-.2);
                                    robot.fl.setPower(.2);
                                    robot.fr.setPower(-.2);
                                    robot.br.setPower(.2);
                                }
                            } else if (ratio > 5 && ratio < 10) {
                                break;
                            }*/

                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());

                                if (recognition.getLabel() == "Skystone") {
                                    int firstforward = 126;

                                    //pick it up
                                    pickup(1); //pick up skystone
                                    sleep(100);
                                    encoderDrive(0.3, -0.3, -0.3, 0.3, 0.3, 100); //move stone to the end
                                    sleep(100);
                                    pickup(0); //release the servo
                                    sleep(100);
                                    encoderDrive(0.3, 0.3, 0.3, -0.3, -0.3, 50);
                                    sleep(100);
                                    //pick it up
                                    pickup(1); //pick up second skystone
                                    sleep(100);
                                    encoderDrive(0.3, -0.3, -0.3, 0.3, 0.3, 50); //move to the end
                                    sleep(100);
                                    pickup(0); //release the servo
                                    sleep(100);
                                    break;

                                } else {
                                    encoderDrive(-0.5, -0.3,-0.3,0.3,0.3,5);
                                }


                                if (recognition.getLabel() == "Skystone") {
                                    int firstforward = 118;
                                    //pick it up
                                    pickup(1); //pick up skystone
                                    sleep(100);
                                    encoderDrive(0.3, -0.3, -0.3, 0.3, 0.3, 40); //move stone to the end
                                    sleep(100);
                                    pickup(0); //release the servo
                                    sleep(100);
                                    encoderDrive(0.3,0.3,0.3,-0.3,-0.3, 20);
                                    sleep(100);
                                    //pick it up
                                    pickup(1); //pick up second skystone
                                    sleep(100);
                                    encoderDrive(-0.3, -0.3,-0.3,0.3, 0.3, 20); //move to the end
                                    sleep(100);
                                    pickup(0); //release the servo
                                    sleep(100);
                                    break;
                                } else {
                                    encoderDrive(-0.5, -0.3, 0.3, 0.3, 0.3, 5);
                                }


                                if (recognition.getLabel() == "Skystone") {
                                    int firstforward = 110;

                                    //pick it up
                                    pickup(1); //pick up skystone
                                    sleep(100);
                                    encoderDrive(0.3, -0.3,-0.3,0.3,0.3, 10); //move stone to the end
                                    sleep(100);
                                    pickup(0); //release the servo
                                    sleep(100);
                                    encoderDrive(0.3, 0.3,0.3,-0.3,-0.3, 5);
                                    sleep(100);
                                    //pick it up
                                    pickup(1); //pick up second skystone
                                    sleep(100);
                                    encoderDrive(0.3, -0.3,-0.3,0.3,0.3, 5); //move to the end
                                    sleep(100);
                                    pickup(0); //release the servo
                                    sleep(100);
                                    break;
                                }

                            }
                            telemetry.update();
                        }
                    }
                }
            }

            if (tfod != null) {
                tfod.shutdown();
            }
        }
    }

            /**
             * Initialize the Vuforia localization engine.
             */
            private void initVuforia () {
                /*
                 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
                 */
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

                parameters.vuforiaLicenseKey = VUFORIA_KEY;
                parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

                //  Instantiate the Vuforia engine
                vuforia = ClassFactory.getInstance().createVuforia(parameters);

                // Loading trackables is not necessary for the TensorFlow Object Detection engine.
            }

            /**
             * Initialize the TensorFlow Object Detection engine.
             */
            private void initTfod () {
                int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                        "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
                tfodParameters.minimumConfidence = 0.8;
                tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
                tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
            }
            public void encoderDrive ( double speed,
            double leftInches, double rightInches, double leftBackInches,
            double rightBackInches,
            double timeoutS){
                robot.fl.setPower(leftInches);
                robot.bl.setPower(leftBackInches);
                robot.fr.setPower(rightInches);
                robot.br.setPower(rightBackInches);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
                    telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                }

                // Step 4:  Stop and close the claw.
                robot.fl.setPower(0);
                robot.bl.setPower(0);
                robot.fr.setPower(0);
                robot.br.setPower(0);

                telemetry.addData("Path", "Complete");
                telemetry.update();
                sleep(1000);
            }
            public void pickup ( double position){
                robot.drag.setPosition(position);
            }
        }

