package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "FoundationRed")
public class FoundationRed extends LinearOpMode {
    componentsHardCode robot = new componentsHardCode();
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        waitForStart();
        if(opModeIsActive()) {
            //keep moving until it hits
            while (robot.touch.getState() == true) {
                moveBackWard(0.4, 75, 0);
                if (robot.touch.getState() == false) {
                    stopRoobot();
                    break;
                }
            }
            sleep(500);
            //strafe to the middle
            strafeLeft(1, 700, 500);
            //move to  align
            moveBackWard(.3,500,500);
            //latch on
            robot.foundation.setPower(-0.5);
            Thread.sleep(2000);
            sleep(500);

            // go back
            moveForward(.4,4005,500);
            //
            sleep(1000);
            robot.foundation.setPower(0.5);
            Thread.sleep(2000);
            sleep(500);
            strafeRight(1,700,500);
            moveForward(.4,700,500);
            moveBackWard(.4,1350,500);
            strafeLeft(1,1050,500);
            while(true){
                if(robot.sensorColor.red() < 40){
                    strafeRight(1,75,0);
                }else if(robot.sensorColor.red() < 40){
                    break;
                }
            }




        }

    }
    public void moveForward(double power,int movement,int sleep) throws InterruptedException{
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
    public void moveBackWard(double power,int movement,int sleep) throws InterruptedException{
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
    public void strafeLeft (double power,int movement,int sleep) throws InterruptedException{
        robot.fl.setPower(-power);
        robot.bl.setPower(power);
        robot.fr.setPower(power);
        robot.br.setPower(-power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }
    public void strafeRight(double power,int movement,int sleep) throws InterruptedException{
        robot.fl.setPower(power);
        robot.bl.setPower(-power);
        robot.fr.setPower(-power);
        robot.br.setPower(power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }
    public void stopRoobot(){
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
    }
}

