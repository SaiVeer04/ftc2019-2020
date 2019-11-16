/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

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
@Autonomous(name = "Tensor_Flow", group = "Concept")
//@Disabled
public class Tensorflow_test extends LinearOpMode {

    componentsHardCode robot = new componentsHardCode();
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


        if (opModeIsActive()) {
            //go forward
            moveForward(.4,250,500);
           /*robot.fl.setPower(-.4);
           robot.bl.setPower(.4);
           robot.fr.setPower(-.4);
           robot.br.setPower(.4);
           Thread.sleep(250);
           robot.fl.setPower(0);
           robot.bl.setPower(0);
           robot.fr.setPower(0);
           robot.br.setPower(0);
           sleep(500);*/
           //strafe left
            strafeLeft(.7,1900,500);
            /*robot.fl.setPower(-.7);
            robot.bl.setPower(.7);
            robot.fr.setPower(.7);
            robot.br.setPower(-.7);
            Thread.sleep(1750);
            robot.fl.setPower(0);
            robot.bl.setPower(0);
            robot.fr.setPower(0);
            robot.br.setPower(0);
            sleep(500);*/
            //go backwards to align with wall
            moveBackWard(.4,850,500);
            /*robot.fl.setPower(.4);
            robot.bl.setPower(-.4);
            robot.fr.setPower(.4);
            robot.br.setPower(-.4);
            Thread.sleep(350);
            robot.fl.setPower(0);
            robot.bl.setPower(0);
            robot.fr.setPower(0);
            robot.br.setPower(0);
            sleep(500);*/


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

                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                              double imageHeight = recognition.getImageHeight();
                              double objectHeight= recognition.getHeight();
                              double ratio = imageHeight/objectHeight;
                              double angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                              robot.fl.setPower(0);
                              robot.bl.setPower(0);
                              robot.fr.setPower(0);
                              robot.br.setPower(0);
                              sleep(5000);

                        }
                        telemetry.update();
                    }else if(updatedRecognitions == null){
                        robot.fl.setPower(-.3);
                        robot.bl.setPower(.3);
                        robot.fr.setPower(-.3);
                        robot.br.setPower(.3);
                        Thread.sleep(75);
                        sleep(75);
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
    private void initVuforia() {
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
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    public void encoderDrive  ( double speed,
                               double leftInches, double rightInches, double leftBackInches,
                               double rightBackInches,
                               double timeoutS) throws InterruptedException{
        robot.fl.setPower(leftInches);
        robot.bl.setPower(leftBackInches);
        robot.fr.setPower(rightInches);
        robot.br.setPower(rightBackInches);
        Thread.sleep(1000);
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
    public void moveForward(double power,int movement,int sleep) throws InterruptedException{
        robot.fl.setPower(-power);
        robot.bl.setPower(power);
        robot.fr.setPower(-power);
        robot.br.setPower(power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }
    public void moveBackWard(double power,int movement,int sleep) throws InterruptedException{
        robot.fl.setPower(power);
        robot.bl.setPower(-power);
        robot.fr.setPower(power);
        robot.br.setPower(-power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }
    public void strafeLeft (double power,int movement,int sleep) throws InterruptedException{
        robot.fl.setPower(-power);
        robot.bl.setPower(power);
        robot.fr.setPower(power);
        robot.br.setPower(-power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }
    public void strafeRight(double power,int movement,int sleep) throws InterruptedException{
        robot.fl.setPower(power);
        robot.bl.setPower(-power);
        robot.fr.setPower(-power);
        robot.br.setPower(power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }
    public void stopRoobot(){
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
    }

}
//90 degree turn
 /*robot.fl.setPower(.4);
         robot.bl.setPower(.4);
         robot.fr.setPower(-.4);
         robot.br.setPower(-.4);
         Thread.sleep(2000);
         robot.fl.setPower(0);
         robot.bl.setPower(0);
         robot.fr.setPower(0);
         robot.br.setPower(0);*/