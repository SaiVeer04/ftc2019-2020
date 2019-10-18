package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "mecanum")
public class teleop_test extends LinearOpMode {

    components robot = new components();

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            double games = gamepad1.right_stick_x;
            telemetry.addData("Gamepad:", games);
            telemetry.update();
            
            if(gamepad1.right_stick_x > 0){
                robot.fl.setPower(gamepad1.right_stick_x);
                robot.bl.setPower(-gamepad1.right_stick_x);
                robot.fr.setPower(-gamepad1.right_stick_x);
                robot.br.setPower(gamepad1.right_stick_x);
            }

              else  if(gamepad1.right_stick_x == 0){
                    robot.fl.setPower(0);
                    robot.bl.setPower(0);
                    robot.fr.setPower(0);
                    robot.br.setPower(0);
                }

            else if(gamepad1.right_stick_x < 0){
                robot.fl.setPower(-gamepad1.right_stick_x);
                robot.bl.setPower(gamepad1.right_stick_x);
                robot.fr.setPower(gamepad1.right_stick_x);
                robot.br.setPower(-gamepad1.right_stick_x);
            }

        }
    }

}
