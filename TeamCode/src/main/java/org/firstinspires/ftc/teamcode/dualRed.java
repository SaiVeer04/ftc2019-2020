package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Bin Laden")
public class dualRed extends LinearOpMode {
    componentsHardCode robot = new componentsHardCode();

    float hsvValues[] = {0F, 0F, 0F};

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;


    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart(); //when the driver clicks play

        if (opModeIsActive()) {
            while (robot.sensorRange.getDistance(DistanceUnit.CM) > 2) {
                robot.fl.setPower(0.7);
                robot.fr.setPower(0.7);
                robot.bl.setPower(-0.7);
                robot.br.setPower(-0.7);
                if (robot.sensorRange.getDistance(DistanceUnit.CM) <= 2) {
                    stopRobot();
                    break;
                }
            }


            for (int i = 0; i <= 12; i++) {
                sleep(500);
                Color.RGBToHSV((int) (robot.colorSensor.red() * SCALE_FACTOR),
                        (int) (robot.colorSensor.green() * SCALE_FACTOR),
                        (int) (robot.colorSensor.blue() * SCALE_FACTOR),
                        hsvValues);
                sleep(500);

                if (hsvValues[0] > 80) { //check the hue value

                    strafeRight(0.5, 200, 20); //to align properly
                    moveForward(0.5, 50, 200);
                    robot.intakeLeft.setPower(-0.7); //make the intake wheels move
                    robot.intakeRight.setPower(0.7);
                    moveBackward(0.2, 1200, 400); //move the robot
                    robot.intakeLeft.setPower(0); //stop the intake wheels
                    robot.intakeRight.setPower(0);
                    break;

                } else {
                    strafeRight(0.5, 250, 20);
                    robot.fl.setPower(-0.5); //to turn
                    robot.fr.setPower(0.5);
                    robot.bl.setPower(0.5);
                    robot.br.setPower(-0.5);
                    sleep(10);
                    robot.fl.setPower(0);
                    robot.bl.setPower(0);
                    robot.fr.setPower(0);
                    robot.br.setPower(0);
                }
            }

            moveForward(0.5, 1000, 200); //move backward

            robot.fl.setPower(-0.5); //to turn
            robot.fr.setPower(0.5);
            robot.bl.setPower(0.5);
            robot.br.setPower(-0.5);
            sleep(800);
            robot.fl.setPower(0);
            robot.bl.setPower(0);
            robot.fr.setPower(0);
            robot.br.setPower(0);

            moveForward(0.5, 2500, 200); //move past the line

            robot.intakeLeft.setPower(0.7); //make the intake wheels move
            robot.intakeRight.setPower(-0.7);
            sleep(300);
            robot.intakeLeft.setPower(0); //stop the intake wheels
            robot.intakeRight.setPower(0);


            strafeRight(0.5, 1200, 200); //move away from the block

            while (robot.bottomSensor.blue() < 200) {
                moveBackward(0.3, 350, 10);
                if (robot.bottomSensor.blue() > 75) {
                    stopRobot();
                    break;
                }


            }

            //SECOND PART
            sleep(1000);

            while (robot.sensorRange1.getDistance(DistanceUnit.CM) > 2) {
                moveBackward(0.2, 200, 0);
                if (robot.sensorRange1.getDistance(DistanceUnit.CM) <= 2) {
                    stopRobot();
                    break;
                }
            }

            strafeLeft(0.3, 1000, 200); //strafe to the middle

            robot.fl.setPower(0.3);
            robot.bl.setPower(-0.3);
            robot.fr.setPower(-0.3);
            robot.br.setPower(0.3);
            sleep(50);
            robot.fl.setPower(0);
            robot.bl.setPower(0);
            robot.fr.setPower(0);
            robot.br.setPower(0);

            /*while (robot.sensorRange1.getDistance(DistanceUnit.CM) > 1) { //move up
                moveBackward(0.2, 100, 0);
                if (robot.sensorRange1.getDistance(DistanceUnit.CM) <= 1) {
                    stopRobot();
                    break;
                }
            }*/

            //distanceCheck();

            moveBackward(.4,300,200); //move closer to the robot because strafing moves ht robot backward

            /*while (robot.sensorRange1.getDistance(DistanceUnit.CM) > 2) { //move up
                moveBackward(0.2, 100, 0);
                if (robot.sensorRange1.getDistance(DistanceUnit.CM) <= 2) {
                    stopRobot();
                    break;
                }
            }*/


            robot.leftServo.setPosition(-0.5); //clamp on
            robot.rightServo.setPosition(0.4);
            sleep(2500);


            moveForward(0.4, 3000, 200); //move backward

            robot.leftServo.setPosition(1); //let go
            robot.rightServo.setPosition(0);
            sleep(300);

            moveBackward(0.5, 100, 500); //move up a little to avoid friction


            strafeRight(0.5, 2500, 200); //move parallel

            moveBackward(0.5, 500, 500); //move to the side

            strafeLeft(0.5, 2500, 500); //push the foundation in

            moveForward(0.3, 400, 200); //move backward a bit


        }

    }

    public void moveForward(double power, int movement, int sleep) throws InterruptedException {
        robot.fl.setPower(-power);
        robot.fr.setPower(-power);
        robot.bl.setPower(power);
        robot.br.setPower(power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }

    public void moveBackward(double power, int movement, int sleep) throws InterruptedException {
        robot.fl.setPower(power);
        robot.fr.setPower(power);
        robot.bl.setPower(-power);
        robot.br.setPower(-power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }

    public void strafeLeft(double power, int movement, int sleep) throws InterruptedException {
        robot.fl.setPower(+power);
        robot.fr.setPower(-power);
        robot.bl.setPower(+power);
        robot.br.setPower(-power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }

    public void strafeRight(double power, int movement, int sleep) throws InterruptedException {
        robot.fl.setPower(-power);
        robot.fr.setPower(+power);
        robot.bl.setPower(-power);
        robot.br.setPower(+power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }

    public void stopRobot() {
        robot.fl.setPower(0);
        robot.fr.setPower(0);
        robot.bl.setPower(0);
        robot.br.setPower(0);
    }

    public void distanceCheck() throws InterruptedException {
        while (robot.sensorRange.getDistance(DistanceUnit.CM) != robot.sensorRange1.getDistance(DistanceUnit.CM)) {
            if (robot.sensorRange.getDistance(DistanceUnit.CM) > robot.sensorRange1.getDistance(DistanceUnit.CM)) {
                robot.fl.setPower(0.1); //turn left
                robot.bl.setPower(-0.1);
                robot.fr.setPower(-0.1);
                robot.br.setPower(0.1);
                Thread.sleep(100);
                robot.fl.setPower(0);
                robot.bl.setPower(0);
                robot.fr.setPower(0);
                robot.br.setPower(0);
            } else if (robot.sensorRange.getDistance(DistanceUnit.CM) < robot.sensorRange1.getDistance(DistanceUnit.CM)) {
                robot.fl.setPower(-0.1); //turn right
                robot.bl.setPower(0.1);
                robot.fr.setPower(0.1);
                robot.br.setPower(-0.1);
                Thread.sleep(100);
                robot.fl.setPower(0);
                robot.bl.setPower(0);
                robot.fr.setPower(0);
                robot.br.setPower(0);
            } else {
                break;
            }

        }
    }
}
