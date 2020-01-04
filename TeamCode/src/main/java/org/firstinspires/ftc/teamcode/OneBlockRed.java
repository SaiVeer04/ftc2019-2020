package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "OneBlockRed")
public class OneBlockRed extends LinearOpMode {
    componentsEncoder robot = new componentsEncoder();
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        while (robot.touchSensor.getState() == true) {
            robot.forward(3,0.3);

            if (robot.touchSensor.getState() == false) {
                robot.fl.setPower(0);
                robot.fr.setPower(0);
                robot.bl.setPower(0);
                robot.br.setPower(0);
                break;
            }
        }


        for (int i = 0; i <3; i++) {
            if (robot.colorSensor.red() == 0 && robot.colorSensor.blue() == 0 && robot.colorSensor.green() == 0) {
                robot.forward(5, 0.7); //move forward to the stones
                robot.intakeLeft.setPower(1); //rotate to take the stone in
                robot.intakeRight.setPower(1);
                sleep(500);
                robot.intakeLeft.setPower(0);
                robot.intakeRight.setPower(0);
                robot.backwards(6, 0.7); //move backwards
                break; //exit the loop
            } else {
                robot.straferight(4, 0.7);
            }
        }

        robot.strafeleft(20, 0.7); //move all the way to the other side

        while (robot.touchSensor.getState() == true) {
            robot.forward(3,0.3);

            if (robot.touchSensor.getState() == false) {
                robot.fl.setPower(0);
                robot.fr.setPower(0);
                robot.bl.setPower(0);
                robot.br.setPower(0);
                break;
            }
        }
        robot.string.setPower(1); //move the mechanism up
        sleep(300);
        robot.string.setPower(0);

        robot.rotate.setPower(1); //turn the mechanism around
        sleep(500);
        robot.rotate.setPower(0);

        robot.string.setPower(-1); //move the mechanism down
        sleep(300);
        robot.string.setPower(0);

        robot.string.setPower(1); //move the mechanism up
        sleep(300);
        robot.string.setPower(0);

        robot.rotate.setPower(-1); //turn the mechanism around
        sleep(500);
        robot.rotate.setPower(0);

        robot.string.setPower(-1); //move the mechanism down
        sleep(300);
        robot.string.setPower(0);

        robot.leftServo.setPosition(1); //move clamps down
        robot.rightServo.setPosition(1);

        robot.turn(90, "l", 0.7); //turn left

        robot.leftServo.setPosition(0); //move clamps up
        robot.rightServo.setPosition(0);

        while (robot.colorSensor.red() < 40) {
            robot.fl.setPower(-1); //move backward
            robot.fr.setPower(1);
            robot.bl.setPower(-1);
            robot.br.setPower(1);
        }






    }
}

