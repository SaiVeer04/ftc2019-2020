package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "FoundationBlueRight")
public class FoundationBlueRight extends LinearOpMode {
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

        robot.leftServo.setPosition(1); //move clamps down
        robot.rightServo.setPosition(1);

        robot.turn(90, "r", 0.7); //turn left

        robot.leftServo.setPosition(0); //move clamps up
        robot.rightServo.setPosition(0);

        robot.straferight(5, 0.7); //puts you on the other side of the parking side

        while (robot.colorSensor.red() < 40) {
            robot.fl.setPower(-1); //move backward
            robot.fr.setPower(1);
            robot.bl.setPower(-1);
            robot.br.setPower(1);
        }
    }
}

