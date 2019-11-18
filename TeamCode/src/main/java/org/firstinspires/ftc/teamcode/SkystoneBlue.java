package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "SkystoneBlue")
public class SkystoneBlue extends LinearOpMode {
    componentsHardCode robot = new componentsHardCode();
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        waitForStart();
        //got to block
        strafeRight(.7,1600,500);
        //clamp block
        robot.drag.setPosition(0);
        //strafe out
        strafeLeft(.7,800,500);
        //move forward until red
        while(true){
            if(robot.sensorColor.blue() <= 40){
                moveBackWard(.3,75,75);

            }else if(robot.sensorColor.blue() > 40){
                break;
            }
        }
        //move extra just in case
        moveBackWard(.4,1000,500);
        //drop
        robot.drag.setPosition(0);
        //then park
        moveForward(.4,1000,500);


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
