package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 10/22/2016.
 */

public abstract class AutonomousBase extends VelocityBase {

    final float THROWING_TIME = 0.2f;
    public int stateIndex = 0;

    @Override
    public void loop() {

        telemetry.addData("currentState: ", currentState);

        switch (currentState) {

            case STATE_INITIAL:

                startPath(launchPositioningPath);
                switchToNextState();

                break;

            case STATE_POSITION_FOR_BALL:

                if (pathComplete()) {

                    TurnOffAllDriveMotors();
                    elapsedTimeForCurrentState.reset();
                    switchToNextState();
                }

                break;

            case STATE_LAUNCH_FIRST_BALL:

                throwBall(elapsedTimeForCurrentState, THROWING_TIME);

                if (elapsedTimeForCurrentState.time() >= 3.0f) {

                    collectionGateServo.setPosition(GATE_SERVO_POSITION_LOW);
                    elapsedTimeForCurrentState.reset();
                    switchToNextState();
                }

                break;

            case STATE_LOAD_BALL:

                if (elapsedTimeForCurrentState.time() >= 1.0f) {

                    elapsedTimeForCurrentState.reset();
                    switchToNextState();
                }

                break;

            case STATE_LAUNCH_SECOND_BALL:

                throwBall(elapsedTimeForCurrentState, THROWING_TIME);

                if (elapsedTimeForCurrentState.time() >= 3.0f) {

                    startPath(beaconPath);
                    switchToNextState();
                }

                break;

            case STATE_DRIVE:

                if (pathComplete()) {

                    TurnOffAllDriveMotors();
                    switchToNextState();
                }

                telemetry.addData("RightPosition", getRightPosition());
                telemetry.addData("RightTarget", currentEncoderTargets.frontRightTarget);
                telemetry.addData("LeftPosition", getLeftPosition());
                telemetry.addData("LeftTarget", currentEncoderTargets.frontLeftTarget);

                break;

            case STATE_FIND_WHITE_LINE:

                if (lineSensor.getRawLightDetected() >= 1.0f) {

                    elapsedTimeForCurrentState.reset();
                    TurnOffAllDriveMotors();
                    switchToNextState();

                } else {

                    if (isRed()) {

                        setPowerForMecanumStrafe(-1.0f);

                    } else {

                        setPowerForMecanumStrafe(1.0f);
                    }

                    setMotorPowerLevels(powerLevels);
                }

                break;

            case STATE_PUSH_BEACON:

                if (elapsedTimeForCurrentState.time() <= 3.0f) {

                    if (isRed()) {

                        pushBeaconButton(leftBeaconSensor.red(), rightBeaconSensor.red());

                    } else {

                        pushBeaconButton(leftBeaconSensor.blue(), rightBeaconSensor.blue());
                    }

                } else {

                    rightBeaconServo.setPosition(0.0f);
                    leftBeaconServo.setPosition(0.0f);
                    switchToNextState();
                }

                break;

            case STATE_STOP:

                break;
        }
    }

    public void switchToNextState() {

        stateIndex++;

        if (stateIndex >= stateList().length) {

            stateIndex = stateList().length - 1;
        }

        if (stateIndex < 0) {

            stateIndex = 0;
        }

        currentState = stateList()[stateIndex];
    }

    public abstract boolean isRed();

    public abstract State[] stateList();
}
