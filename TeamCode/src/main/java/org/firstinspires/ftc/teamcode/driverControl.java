/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import android.graphics.Paint;
import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="driveControl", group="Iterative Opmode")
public class driverControl extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private static MediaPlayer mediaPlayer = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private Servo StoneServoRight;  // close and open
    private Servo StoneServoLeft;   // close and open
    private DcMotor stoneLift;
    private DcMotor stoneTilt;
    private DcMotor autonStoneExt;
    private DcMotor autonStoneLift;
    private CRServo autonStoneServo;
    private Servo CapStoneServoLock;
    private CRServo autonPlatformServo;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        stoneLift = hardwareMap.get(DcMotor.class, "motor5");
        stoneTilt = hardwareMap.get(DcMotor.class, "motor6");
        autonStoneExt = hardwareMap.get(DcMotor.class, "motor7");
        autonStoneLift = hardwareMap.get(DcMotor.class, "motor8");
        StoneServoLeft = hardwareMap.servo.get("servo1");
        StoneServoRight = hardwareMap.servo.get("servo2");
        CapStoneServoLock = hardwareMap.servo.get("servo4");
        autonStoneServo = hardwareMap.crservo.get("servo6");
        autonPlatformServo = hardwareMap.crservo.get("servo5");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stoneTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stoneLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autonStoneExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autonStoneLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        StoneServoRight.setPosition(0.7);
        StoneServoLeft.setPosition(0.3);
        CapStoneServoLock.setPosition(0.0);
        autonStoneServo.setPower(0);
        autonPlatformServo.setPower(0.0);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double leftFrontPower;
            double rightFrontPower;
            double leftBackPower;
            double rightBackPower;

            while(gamepad2.left_trigger > 0) {
                autonStoneServo.setPower(0.5);
            }
            while(gamepad2.right_trigger > 0){
                autonStoneServo.setPower(-0.5);
            }
            autonStoneServo.setPower(0.0);

            while (gamepad1.dpad_left) {
                autonPlatformServo.setPower(0.5);
            }
            while (gamepad1.dpad_right) {
                autonPlatformServo.setPower(-0.5);
            }
            autonPlatformServo.setPower(0.0);

            if(gamepad2.a){    //lock
                CapStoneServoLock.setPosition(0.0);
            }
            if(gamepad2.b){ //drop
                CapStoneServoLock.setPosition(1.0);
            }

            // Stone Servo Controller Code
            if(gamepad1.left_bumper){    //open
                StoneServoRight.setPosition(0.75); //was .55
                StoneServoLeft.setPosition(0.25); //was .45
            }
            if(gamepad1.right_bumper){ //close
                StoneServoRight.setPosition(1.0);
                StoneServoLeft.setPosition(0.0);
            }
            while(gamepad1.y){
                stoneLift.setPower(0.50);
            }
            while(gamepad1.a){
                stoneLift.setPower(-0.50);
            }
            stoneLift.setPower(0.0);

            while(gamepad1.x){
                stoneTilt.setPower(0.50);
            }
            while(gamepad1.b){
                stoneTilt.setPower(-1.00);
            }
            stoneTilt.setPower(-0.1);

            while(gamepad2.dpad_left){
                autonStoneExt.setPower(0.50);
            }
            while(gamepad2.dpad_right){
                autonStoneExt.setPower(-0.50);
            }
            autonStoneExt.setPower(0.0);

            while(gamepad2.dpad_up){
                autonStoneLift.setPower(0.50);
            }
            while(gamepad2.dpad_down){
                autonStoneLift.setPower(-0.50);
            }
            autonStoneLift.setPower(0.0);
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            float strafe_y = 0;
            float strafe_x = 0;
            float turn = 0;


//            other way{
//                strafe_y = gamepad1.left_stick_y;
//                strafe_x = -gamepad1.left_stick_x;
//                turn = gamepad1.right_stick_x;
//            }
            strafe_y = -gamepad1.left_stick_y;
            strafe_x = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;

            leftFrontPower = Range.clip(strafe_y + strafe_x + turn, -1.0, 1.0);
            rightFrontPower = Range.clip(strafe_y - strafe_x - turn, -1.0, 1.0);
            leftBackPower = Range.clip(strafe_y - strafe_x + turn, -1.0, 1.0);
            rightBackPower = Range.clip(strafe_y + strafe_x - turn, -1.0, 1.0);

            // Send calculated power to wheels
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }


}