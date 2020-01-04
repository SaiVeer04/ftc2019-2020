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
            robot.forward(10,.2);
            robot.backwards(10,.2);
            //keep moving until it hits
            /*while (robot.touchSensor.getState() == true) {
                robot.forward(3, 0.5);
                telemetry.addData("Digital Touch", "Is Not Pressed");
            }*/


            /*robot.leftServo.setPosition(0); //move clamps down
            robot.rightServo.setPosition(0);
            sleep(200);*/

          /*  robot.turn(90, "l", 0.7); //turn left
            sleep(200);

            robot.leftServo.setPosition(0); //move clamps up
            robot.rightServo.setPosition(0);

            while (robot.bottomSensor.red() < 1) {
                robot.fl.setPower(-1); //move backward
                robot.fr.setPower(1);
                robot.bl.setPower(-1);
                robot.br.setPower(1);
            }
            sleep(200);

            robot.fl.setPower(0); //stop
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.br.setPower(0);*/
        }
    }



