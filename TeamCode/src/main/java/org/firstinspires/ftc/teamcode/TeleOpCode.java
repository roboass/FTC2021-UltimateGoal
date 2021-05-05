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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOpCode", group="Linear Opmode")
//@Disabled
public class TeleOpCode extends UsefulFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Initialise();

        telemetry.addData("Launch Servo position", launchServo.getPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean dpadupLock = false, dpaddownLock = false;
        boolean ylock = false, alock = false;
        boolean rightBumperLock = false;
        boolean leftBumper2Lock = false, leftBumper2ModeActive = false;

        while (opModeIsActive()) {
            TeleOpDrive();

            if(gamepad2.right_trigger >= 0.9f) {
                launchMotor.setPower(1);
            } else {
                launchMotor.setPower(0);
            }


            if(gamepad2.right_bumper) {
                if(!rightBumperLock) {
                    rightBumperLock = true;
                    launchServoThread.start();
                }

            }
            else if(rightBumperLock) {
                rightBumperLock = false;
            }

            if(gamepad2.left_bumper) {
                if(!leftBumper2Lock && !leftBumper2ModeActive) {
                    leftBumper2ModeActive = true;

                    GrabClawState(true);
                }else if(!leftBumper2Lock && leftBumper2ModeActive) {
                    leftBumper2ModeActive = false;

                    GrabClawState(false);
                }
                leftBumper2Lock = true;
            } else if(leftBumper2Lock) { ///h
                leftBumper2Lock = false;
            }

            if(gamepad2.a) {
                if(!alock) {
                    alock = true;
                    LiftClawState(currentClawState - 1);
                }
            } else if(alock) {
                alock = false;
            }

            if(gamepad2.y) {
                if(!ylock) {
                    ylock = true;
                    LiftClawState(currentClawState + 1);
                }
            } else if(ylock) {
                ylock = false;
            }

            if(gamepad2.dpad_up) {
                if(!dpadupLock) {
                    dpadupLock = true;
                    AddToLaunchAngle(5);
                }
            } else if(dpadupLock) {
                dpadupLock = false;
            }

            if(gamepad2.dpad_down) {
                if(!dpaddownLock) {
                    dpaddownLock = true;
                    AddToLaunchAngle(-5);
                }
            } else if(dpaddownLock) {
                dpaddownLock = false;
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Current launch angle:", currentLaunchAngle);
            telemetry.update();
        }
        //AddToLaunchAngle(currentLaunchAngle);

    }
}
