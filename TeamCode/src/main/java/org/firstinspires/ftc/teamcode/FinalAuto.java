package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public void runOpMode() {
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
        encoderDrive(.4, -29, -29, -29, -29, 3); //move out
        sleep(200);
        encoderDrive(.3, 8, -8, -8, 8,3); //strafe left
        robot.foundation.setPower(1);
        sleep(200);
        robot.foundation.setPower(0);
        sleep(200);
        encoderDrive(.2, 29, 29, 29, 29, 3); //move back
        sleep(200);
        encoderDrive(.3, 20, -20, -20, 20, 3); //strafe right
        sleep(1000);
        encoderDrive(.3, -3,-3,-3,-3,3); //move backward a little bit
        sleep(200);
        encoderDrive(0.3,2,-2,2,-2, 3);


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

                            if (ratio < 5 && ratio > 10) {
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
                            }

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
                                encoderDrive(0.3, firstforward, firstforward, firstforward, firstforward,0.5); //move stone to the end
                                sleep(100);
                                pickup(0); //release the servo
                                sleep(100);
                                encoderDrive(0.3, -(firstforward-24), -(firstforward-24),-(firstforward-24),-(firstforward-24), 3);
                                sleep(100);
                                //pick it up
                                pickup(1); //pick up second skystone
                                sleep(100);
                                encoderDrive(0.3,firstforward-24, firstforward-24,firstforward-24,firstforward-24, 3); //move to the end
                                sleep(100);
                                pickup(0); //release the servo
                                sleep(100);
                                break;

                            } else {
                                encoderDrive(-0.5, 8,8,8,8,3);
                            }


                            if (recognition.getLabel() == "Skystone") {
                                int firstforward = 118;

                                //pick it up
                                pickup(1); //pick up skystone
                                sleep(100);
                                encoderDrive(0.3, firstforward, firstforward, firstforward, firstforward,0.5); //move stone to the end
                                sleep(100);
                                pickup(0); //release the servo
                                sleep(100);
                                encoderDrive(0.3, -(firstforward-24), -(firstforward-24),-(firstforward-24),-(firstforward-24), 3);
                                sleep(100);
                                //pick it up
                                pickup(1); //pick up second skystone
                                sleep(100);
                                encoderDrive(0.3,firstforward-24, firstforward-24,firstforward-24,firstforward-24, 3); //move to the end
                                sleep(100);
                                pickup(0); //release the servo
                                sleep(100);
                                break;
                            } else {
                                encoderDrive(-0.5, 8,8,8,8,3);
                            }


                            if (recognition.getLabel() == "Skystone") {
                                int firstforward = 110;

                                //pick it up
                                pickup(1); //pick up skystone
                                sleep(100);
                                encoderDrive(0.3, firstforward, firstforward, firstforward, firstforward,0.5); //move stone to the end
                                sleep(100);
                                pickup(0); //release the servo
                                sleep(100);
                                encoderDrive(0.3, -(firstforward-24), -(firstforward-24),-(firstforward-24),-(firstforward-24), 3);
                                sleep(100);
                                //pick it up
                                pickup(1); //pick up second skystone
                                sleep(100);
                                encoderDrive(0.3,firstforward-24, firstforward-24,firstforward-24,firstforward-24, 3); //move to the end
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
                                    int newLeftTarget;
                                    int newLeftBackTarget;
                                    int newRightTarget;
                                    int newRightBackTarget;

                                    // Ensure that the opmode is still active
                                    if (opModeIsActive()) {

                                        // Determine new target position, and pass to motor controller
                                        newLeftTarget = robot.fl.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                                        newRightTarget = robot.fr.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                                        newLeftBackTarget = robot.bl.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
                                        newRightBackTarget = robot.br.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);

                                        robot.fl.setTargetPosition(newLeftTarget);
                                        robot.fr.setTargetPosition(newRightTarget);
                                        robot.bl.setTargetPosition(newLeftBackTarget);
                                        robot.br.setTargetPosition(newRightBackTarget);

                                        // Turn On RUN_TO_POSITION
                                        robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                        robot.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                        robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                        robot.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                                        // reset the timeout time and start motion.
                                        runtime.reset();
                                        robot.fl.setPower(Math.abs(speed));
                                        robot.bl.setPower(Math.abs(speed));
                                        robot.fr.setPower(Math.abs(speed));
                                        robot.br.setPower(Math.abs(speed));


                                        // keep looping while we are still active, and there is time left, and both motors are running.
                                        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                                        // its target position, the motion will stop.  This is "safer" in the event that the robot will
                                        // always end the motion as soon as possible.
                                        // However, if you require that BOTH motors have finished their moves before the robot continues
                                        // onto the next step, use (isBusy() || isBusy()) in the loop test.
                                        while (opModeIsActive() &&
                                                (runtime.seconds() < timeoutS) &&
                                                (robot.fl.isBusy() && robot.fr.isBusy() && robot.bl.isBusy() && robot.br.isBusy())) {

                                            // Display it for the driver.
                                            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget, newLeftBackTarget, newRightBackTarget);
                                            telemetry.addData("Path2", "Running at %7d :%7d",
                                                    robot.fl.getCurrentPosition(),
                                                    robot.bl.getCurrentPosition(),
                                                    robot.fr.getCurrentPosition(),
                                                    robot.br.getCurrentPosition());
                                            telemetry.update();
                                        }

                                        // Stop all motion;
                                        robot.fl.setPower(0);
                                        robot.bl.setPower(0);
                                        robot.fr.setPower(0);
                                        robot.br.setPower(0);

                                        // Turn off RUN_TO_POSITION
                                        robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                        robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                        robot.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                                        //  sleep(250);   // optional pause after each move
                                    }
                                }
    public void pickup(double position) {
        robot.drag.setPosition(position);
    }
                            }
