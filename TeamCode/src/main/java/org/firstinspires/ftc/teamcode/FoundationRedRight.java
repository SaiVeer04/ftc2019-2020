package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "FoundationRedRight")
public class FoundationRedRight extends LinearOpMode {
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

        robot.leftServo.setPosition(1); //move clamps down
        robot.rightServo.setPosition(1);

        robot.turn(90, "l", 0.7); //turn right

        robot.leftServo.setPosition(0); //move clamps up
        robot.rightServo.setPosition(0);

        robot.strafeleft(5, 0.7); //puts you on the other side of the parking side

        while (robot.colorSensor.red() < 40) {
            robot.fl.setPower(-1); //move backward
            robot.fr.setPower(1);
            robot.bl.setPower(-1);
            robot.br.setPower(1);
        }
    }
}

