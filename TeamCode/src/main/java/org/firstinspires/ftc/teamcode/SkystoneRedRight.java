package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "SkystoneRedRight")
public class SkystoneRedRight extends LinearOpMode {
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

        while (robot.bottomSensor.red() < 1) { //move to the other side by strafing left
            robot.fl.setPower(1);
            robot.fr.setPower(1);
            robot.bl.setPower(-1);
            robot.br.setPower(-1);
        }

        robot.turn(90, "l", 0.5); //turn to drop the stone past the line


        robot.intakeLeft.setPower(-1); //drop the stone
        robot.intakeRight.setPower(-1);
        sleep(500);
        robot.intakeLeft.setPower(0);
        robot.intakeRight.setPower(0);

        robot.turn(90, "r", 0.5); //back to original position
        robot.straferight(30, 0.7); //strafe all the way to the other side

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
                robot.backwards(6, 0.7); //move backwards (THIS IS THE VALUE THAT DETERMINES LEFT OR RIGHT)
                break; //exit the loop
            } else {
                robot.strafeleft(4, 0.7);
            }
        }

        while (robot.bottomSensor.red() < 1) { //move to the other side by strafing left
            robot.fl.setPower(1);
            robot.fr.setPower(1);
            robot.bl.setPower(-1);
            robot.br.setPower(-1);
        }

        robot.turn(90, "l", 0.5); //turn to drop the stone

        robot.intakeLeft.setPower(-1); //drop the stone
        robot.intakeRight.setPower(-1);
        sleep(500);
        robot.intakeLeft.setPower(0);
        robot.intakeRight.setPower(0);




    }
}

