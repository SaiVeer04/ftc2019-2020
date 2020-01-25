package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "FoundationRedRight")
public class SRSTest1 extends LinearOpMode {
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

            //distanceCheck();

            moveBackward(.4,200,200); //move closer to the robot because strafing moves ht robot backward

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

            strafeRight(0.5, 2500, 200); //move parallel

            moveBackward(0.5, 500, 1000); //move to the side

            strafeLeft(0.5, 2000, 500); //push the foundation in

            moveBackward(0.3, 600, 200); //move backward a bit



            sleep(600);



            while (robot.bottomSensor.red() < 350) {
                strafeRight(0.6, 350, 10);

                if (robot.bottomSensor.red() > 100) {
                    stopRobot();
                    break;
                }


            }
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

