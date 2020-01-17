package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "SRSTest")
public class SRSTest extends LinearOpMode {
    componentsHardCode robot = new componentsHardCode();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart(); //when the driver clicks play


        if (opModeIsActive()) {

            while (robot.sensorRange1.getDistance(DistanceUnit.CM) > 2) {
                moveBackward(0.2, 200, 0);
                if (robot.sensorRange1.getDistance(DistanceUnit.CM) <= 2) {
                    stopRobot();
                    break;
                }
            }

            strafeLeft(0.3, 1000, 200); //strafe to the middle

            /*while (robot.sensorRange1.getDistance(DistanceUnit.CM) > 1) { //move up
                moveBackward(0.2, 100, 0);
                if (robot.sensorRange1.getDistance(DistanceUnit.CM) <= 1) {
                    stopRobot();
                    break;
                }
            }*/

            distanceCheck();

            moveBackward(.4,200,200);

            /*while (robot.sensorRange1.getDistance(DistanceUnit.CM) > 2) { //move up
                moveBackward(0.2, 100, 0);
                if (robot.sensorRange1.getDistance(DistanceUnit.CM) <= 2) {
                    stopRobot();
                    break;
                }
            }*/


            robot.leftServo.setPower(-0.5); //clamp on
            robot.rightServo.setPower(0.5);
            sleep(2500);
            robot.leftServo.setPower(0);
            robot.rightServo.setPower(0);

            moveForward(0.2, 3500, 200); //move backward

            robot.leftServo.setPower(-0.5); //clamp on
            robot.rightServo.setPower(0.5);
            sleep(750);
            robot.leftServo.setPower(0);
            robot.rightServo.setPower(0);

            moveForward(0.2, 3500, 200); //move backward


            robot.leftServo.setPower(-0.5); //clamp on
            robot.rightServo.setPower(0.5);
            sleep(750);
            robot.leftServo.setPower(0);
            robot.rightServo.setPower(0);

            //strafeLeft(.5,2000,2000);


            /*robot.fl.setPower(0.4); //turn to align
            robot.bl.setPower(-0.4);
            robot.fr.setPower(-0.4);
            robot.br.setPower(0.4);
            Thread.sleep(1200);
            robot.fl.setPower(0);
            robot.bl.setPower(0);
            robot.fr.setPower(0);
            robot.br.setPower(0);*/




            robot.leftServo.setPower(0.5); //let go
            robot.rightServo.setPower(-0.5);
            sleep(2000);
            robot.leftServo.setPower(0);
            robot.rightServo.setPower(0);

            strafeLeft(0.2, 1200, 200);

            moveForward(0.2, 200, 200);

            /*while (robot.bottomSensor.red() < 100) {
                strafeRight(0.5, 4000, 200);

                if (robot.bottomSensor.red() > 100) {
                    stopRobot();
                }


            }*/
        }
    }

    public void moveForward(double power, int movement, int sleep) throws InterruptedException {
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

    public void moveBackward(double power, int movement, int sleep) throws InterruptedException {
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

    public void strafeLeft(double power, int movement, int sleep) throws InterruptedException {
        robot.fl.setPower(+power);
        robot.bl.setPower(+power);
        robot.fr.setPower(-power);
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
        robot.bl.setPower(-power);
        robot.fr.setPower(+power);
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
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
    }

    public void distanceCheck() throws InterruptedException{
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
            }

            else if (robot.sensorRange.getDistance(DistanceUnit.CM) < robot.sensorRange1.getDistance(DistanceUnit.CM)) {
                robot.fl.setPower(-0.1); //turn right
                robot.bl.setPower(0.1);
                robot.fr.setPower(0.1);
                robot.br.setPower(-0.1);
                Thread.sleep(100);
                robot.fl.setPower(0);
                robot.bl.setPower(0);
                robot.fr.setPower(0);
                robot.br.setPower(0);
            }
            else {
                break;
            }
        }
    }
}

