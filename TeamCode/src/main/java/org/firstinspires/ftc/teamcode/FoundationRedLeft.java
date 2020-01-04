package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "FoundationRedLeft")
public class FoundationRedLeft extends LinearOpMode {
    componentsEncoder robot = new componentsEncoder();
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        while (robot.distanceSensor.getDistance(DistanceUnit.CM) > 1) {
            robot.fl.setPower(1); //move forward
            robot.fr.setPower(-1);
            robot.bl.setPower(1);
            robot.br.setPower(-1);
        }

        robot.fl.setPower(0);
        robot.fr.setPower(0);
        robot.bl.setPower(0);
        robot.br.setPower(0);

        robot.leftServo.setPosition(1); //move clamps down
        robot.rightServo.setPosition(1);

        robot.turn(90, "l", 0.7); //turn left

        robot.leftServo.setPosition(0); //move clamps up
        robot.rightServo.setPosition(0);

        while (robot.colorSensor.red() < 1) {
            robot.fl.setPower(-1); //move backward
            robot.fr.setPower(1);
            robot.bl.setPower(-1);
            robot.br.setPower(1);
        }

        robot.fl.setPower(0); //move forward to the stones
        robot.fr.setPower(0);
        robot.bl.setPower(0);
        robot.br.setPower(0);
    }


}

