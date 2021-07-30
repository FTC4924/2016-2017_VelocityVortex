package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 4924_Users on 1/3/2017.
 */

public abstract class RevolutionAutonomousBase extends RevolutionVelocityBase {

    public enum State {

        STATE_INITIAL,
        STATE_DRIVE,
        STATE_STOP,
        STATE_LAUNCH_BALL,
        STATE_POSITION_FOR_BALL,
        STATE_KNOCK_CAP_BALL,
        STATE_FIND_WHITE_LINE,
        STATE_LINE_UP_TO_BEACON,
        STATE_BACK_UP,
        STATE_TURN_TO_ZERO,
        STATE_DRIVE_TO_BEACON,
        STATE_TURN_TO_BEACON,
        STATE_PUSH_BEACON,
        STATE_CHECK_BEACON,
        STATE_CHECK_BEACON_2S,
        STATE_LOAD_BALL,
        STATE_WAIT_FOR_BALL,
        STATE_START_BEACON_PATH,
        STATE_START_LAUNCH_PATH,
        STATE_START_CAP_BALL_PATH,
        STATE_DROP_GATE,
        STATE_SQUARE_ON_WALL,
        STATE_START_RAMP_PATH,
        STATE_DRIVE_TO_RAMP,
        STATE_START_DRIVE_PATH,
        STATE_CORNER_PATH,
        STATE_CRYPTOGRAPH,
    }

    final float THROWING_TIME = 0.5f;
    public int stateIndex = 0;
    public int currentPathSegmentIndex = 0;
    public int lastHeadingDifference = 0;
    public boolean stateStarted = false;
    DrivePathSegment segment = new DrivePathSegment();
    public EncoderTargets zeroEncoderTargets = new EncoderTargets(0, 0);
    EncoderTargets currentEncoderTargets = zeroEncoderTargets;

    public DrivePathSegment[] currentPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };

    public DrivePathSegment[] launchPositioningPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };

    public DrivePathSegment[] beaconPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };
    public DrivePathSegment[] leaveBeaconPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };
    public DrivePathSegment[] knockCapBallPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };
    public DrivePathSegment[] rampPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };
    public DrivePathSegment[] postThrowingPath= new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };

    public DrivePathSegment[] stop = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };

    double countsPerInch = 0.0;
    public ElapsedTime elapsedTimeForCurrentSegment = new ElapsedTime();
    public ElapsedTime elapsedTimeForCurrentState = new ElapsedTime();
    public State currentState;
    static final float TURNING_ANGLE_MARGIN = 7.0f;
    static final int ENCODER_TARGET_MARGIN = 15;
    public static int angleOffset = 0;
    final int COUNTS_PER_REVOLUTION = 1120;
    final double WHEEL_DIAMETER = 4.0f;
    final double GEAR_RATIO = 1.0f;
    final double CALIBRATION_FACTOR = 2.24f; //1.93f;
    public boolean isSecondBeacon = false;

    @Override
    public void init() {

        super.init();
        currentState = State.STATE_INITIAL;
        countsPerInch = (COUNTS_PER_REVOLUTION / (Math.PI * WHEEL_DIAMETER)) * GEAR_RATIO * CALIBRATION_FACTOR;
    }

    @Override
    public void init_loop() {

        rightBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        leftBeaconServo.setPosition(BEACON_SERVO_POSITION_IN);
        collectionGateServo.setPosition(GATE_SERVO_POSITION_CLOSED);
        shovelLockServo.setPosition(LOCK_SERVO_POSITION_CLOSED);
        capBallServo.setPosition(CAP_BALL_SERVO_POSITION_STOPPED);
        ballClampServo.setPosition(BALL_CLAMP_SERVO_POSITION_STOPPED);
    }

    @Override
    public void loop() {

        telemetry.addData("currentState: ", currentState);
        telemetry.addData("segment index", currentPathSegmentIndex);
        telemetry.addData("Gyro", turningGyro.getHeading());

        //telemetry.addData("Target", segment.Angle);
        int heading = turningGyro.getHeading();

        switch (currentState) {

            case STATE_INITIAL:

                    if (!turningGyro.isCalibrating()) {

                        steadyHeading = heading;
                    runWithoutEncoders();
                    switchToNextState();
                }

                break;

            case STATE_START_LAUNCH_PATH:

                startPath(launchPositioningPath);
                switchToNextState();

                break;

            case STATE_POSITION_FOR_BALL:

                if (pathComplete(heading)) {

                    TurnOffAllDriveMotors();
                    switchToNextState();
                }

                break;

            case STATE_DROP_GATE:

                if (elapsedTimeForCurrentState.time() >= 1.0f) {

                    collectionMotor.setPower(0.0f);
                    switchToNextState();

                } else {

                    collectionMotor.setPower(1.0f);
                }

                break;

            case STATE_LAUNCH_BALL:

                throwBall(elapsedTimeForCurrentState, THROWING_TIME);

                if (elapsedTimeForCurrentState.time() >= 0.8f) {

                    switchToNextState();
                }

                break;

            case STATE_LOAD_BALL:

                collectionGateServo.setPosition(1.0f);
                switchToNextState();

                break;

            case STATE_WAIT_FOR_BALL:

                if (elapsedTimeForCurrentState.time() >= 1.0f) {

                    collectionGateServo.setPosition(0.0f);
                    switchToNextState();
                }

                break;

            case STATE_START_BEACON_PATH:

                startPath(beaconPath);
                switchToNextState();

                break;

            case STATE_DRIVE:

                if (pathComplete(heading)) {

                    TurnOffAllDriveMotors();
                    switchToNextState();
                }

                steadyHeading = heading;

                //telemetry.addData("RightPosition", getRightPosition());
                //telemetry.addData("RightTarget", currentEncoderTargets.frontRightTarget);
                //telemetry.addData("LeftPosition", getLeftPosition());
                //telemetry.addData("LeftTarget", currentEncoderTargets.frontLeftTarget);

                break;

            case STATE_FIND_WHITE_LINE:

                if (lineSensor.getRawLightDetected() >= 0.001f) {

                    DbgLog.msg("Line sensor reading: " + lineSensor.getRawLightDetected());
                }

                telemetry.addData("LineSensor", lineSensor.getRawLightDetected());

                if (lineSensor.getRawLightDetected() >= 0.1f) {

                    if (isSecondBeacon) {

                        if (elapsedTimeForCurrentState.time() >= 1.0f) {

                            TurnOffAllDriveMotors();
                            switchToNextState();

                        } else {

                            strafeAgainstWall(heading);
                        }

                    } else {

                        TurnOffAllDriveMotors();
                        switchToNextState();
                    }

                } else {

                    strafeAgainstWall(heading);
                }

                break;

            case STATE_SQUARE_ON_WALL:

                if (!isRed() && isSecondBeacon) {

                    if (elapsedTimeForCurrentState.time() >= 3.0f) {

                        TurnOffAllDriveMotors();
                        switchToNextState();

                    } else {

                        powerLevels.frontRightPower = 0.1f;
                        powerLevels.backRightPower = 0.1f;
                        powerLevels.frontLeftPower = 0.1f;
                        powerLevels.backLeftPower = 0.1f;
                    }

                } else {

                    if (isRed() && isSecondBeacon) {

                        if (elapsedTimeForCurrentState.time() >= 5.0f) {

                            TurnOffAllDriveMotors();
                            switchToNextState();

                        } else {

                            powerLevels.frontRightPower = 0.05f;
                            powerLevels.backRightPower = 0.05f;
                            powerLevels.frontLeftPower = 0.04f;
                            powerLevels.backLeftPower = 0.04f;
                        }

                    } else {

                        if (elapsedTimeForCurrentState.time() >= 1.5f) {

                            TurnOffAllDriveMotors();
                            switchToNextState();

                        } else {

                            powerLevels.frontRightPower = 0.05f;
                            powerLevels.backRightPower = 0.05f;
                            powerLevels.frontLeftPower = 0.04f;
                            powerLevels.backLeftPower = 0.04f;
                        }
                    }
                }

                break;

            case STATE_LINE_UP_TO_BEACON:

                DbgLog.msg("Line sensor reading: " + lineSensor.getRawLightDetected());

                if (lineSensor.getRawLightDetected() >= 0.3f && elapsedTimeForCurrentState.time() >= 0.1f) {

                    TurnOffAllDriveMotors();
                    switchToNextState();

                } else {

                    if (isRed()) {

                        setPowerForMecanumStrafe(0.2f, heading);

                    } else {

                        setPowerForMecanumStrafe(-0.2f, heading);
                    }
                }

                break;

            case STATE_BACK_UP:

                if (!stateStarted) {

                    startPath(new DrivePathSegment[] {

                            new DrivePathSegment(-1.5f, 0.5f, DrivePathSegment.LINEAR),
                    });

                    stateStarted = true;
                }

                if (pathComplete(heading)) {

                    TurnOffAllDriveMotors();
                    switchToNextState();
                }

                break;

            case STATE_TURN_TO_ZERO:

                if (!stateStarted) {

                    startPath(new DrivePathSegment[] {

                            new DrivePathSegment(0.0f, 0.3f, DrivePathSegment.TURN),
                    });

                    stateStarted = true;
                }

                if (pathComplete(heading)) {

                    TurnOffAllDriveMotors();
                    switchToNextState();
                }

                break;

            case STATE_DRIVE_TO_BEACON:

                if (lineSensor.getRawLightDetected() >= 0.1f && elapsedTimeForCurrentState.time() >= 0.5f) {

                    TurnOffAllDriveMotors();
                    switchToNextState();

                } else {

                    setPowerForLinearMove(1.0f);
                }

                break;

            case STATE_TURN_TO_BEACON:

                if (!stateStarted) {

                    if (isRed()) {

                        startPath(new DrivePathSegment[] {

                                new DrivePathSegment(270.0f, 0.3f, DrivePathSegment.TURN),
                        });

                    } else {

                        startPath(new DrivePathSegment[] {

                                new DrivePathSegment(90.0f, 0.3f, DrivePathSegment.TURN),
                        });
                    }

                    stateStarted = true;
                }

                if (pathComplete(heading)) {

                    TurnOffAllDriveMotors();
                    switchToNextState();
                }

                break;

            case STATE_PUSH_BEACON:

                if (elapsedTimeForCurrentState.time() >= 3.0f) {

                    TurnOffAllDriveMotors();
                    switchToNextState();

                } else {

                    setPowerForLinearMove(0.2f);
                }

                break;

            case STATE_CHECK_BEACON:

                boolean beaconIsCorrect;

                if (isRed()) {

                    beaconIsCorrect = leftBeaconSensor.red() >= 3;

                } else {

                    beaconIsCorrect = leftBeaconSensor.blue() >= 3;
                }

                if (beaconIsCorrect) {

                    isSecondBeacon = true;
                    TurnOffAllDriveMotors();
                    switchToNextState();

                } else {

                    if (elapsedTimeForCurrentState.time() >= 3.0f) {
                        new DrivePathSegment(-1.5f, 0.5f, DrivePathSegment.LINEAR);
                        TurnOffAllDriveMotors();
                        restartBeaconSequence();
                    }
                }

                break;

            case STATE_CHECK_BEACON_2S:

                boolean firstBeaconIsCorrect;
                boolean secondBeaconIsCorrect;

                if (isRed()) {

                    firstBeaconIsCorrect = leftBeaconSensor.red() >= 3;
                    secondBeaconIsCorrect = rightBeaconSensor.red() >= 3;

                } else {

                    firstBeaconIsCorrect = leftBeaconSensor.blue() >= 3;
                    secondBeaconIsCorrect = rightBeaconSensor.blue() >= 3;
                }

                if (firstBeaconIsCorrect && secondBeaconIsCorrect) {

                    isSecondBeacon = true;
                    TurnOffAllDriveMotors();
                    switchToNextState();

                } else {

                    TurnOffAllDriveMotors();
                    restartBeaconSequence();
                }

                break;

            case STATE_START_CAP_BALL_PATH:

                startPath(knockCapBallPath);
                switchToNextState();

                break;

            case STATE_KNOCK_CAP_BALL:

                if (pathComplete(heading)) {
                    switchToNextState();
                }

                break;

            case STATE_DRIVE_TO_RAMP:

                startPath(rampPath);
                switchToNextState();

                break;

            case STATE_START_RAMP_PATH:

                if (pathComplete(heading)) {
                    switchToNextState();
                }

                break;

            case STATE_START_DRIVE_PATH:

                startPath(postThrowingPath);
                switchToNextState();

                break;

            case STATE_CORNER_PATH:

                if (pathComplete(heading)) {
                    switchToNextState();
                }

                break;

            case STATE_STOP:

                startPath(stop);
                TurnOffAllDriveMotors();

                break;
        }

        rightBeaconServo.setPosition(rightBeaconServoPosition);
        leftBeaconServo.setPosition(leftBeaconServoPosition);
        setMotorPowerLevels(powerLevels);
        DbgLog.msg("Gyro reading: " + heading);
    }

    public void strafeAgainstWall(int heading) {

        /*if (leftBumper.isPressed()) {

            if (rightBumper.isPressed()) {

                if (isRed()) {

                    if (isSecondBeacon) {

                        setPowerForMecanumStrafe(-0.35f, heading);

                    } else {

                        setPowerForMecanumStrafe(-0.2f, heading);
                    }

                } else {

                    if (isSecondBeacon) {

                        setPowerForMecanumStrafe(0.35f, heading);

                    } else {

                        setPowerForMecanumStrafe(0.2f, heading);
                    }
                }

            } else {

                powerLevels.frontRightPower = 0.2f;
                powerLevels.backRightPower = 0.2f;
                powerLevels.frontLeftPower = 0.0f;
                powerLevels.backLeftPower = 0.0f;
            }

        } else {

            if (rightBumper.isPressed()) {

                powerLevels.frontRightPower = 0.0f;
                powerLevels.backRightPower = 0.0f;
                powerLevels.frontLeftPower = 0.2f;
                powerLevels.backLeftPower = 0.2f;

            } else {

                powerLevels.frontRightPower = 0.2f;
                powerLevels.backRightPower = 0.2f;
                powerLevels.frontLeftPower = 0.2f;
                powerLevels.backLeftPower = 0.2f;
            }
        }*/

        if (isRed()) {

            if (isSecondBeacon) {

                setPowerForMecanumStrafe(-0.5f, heading);

            } else {

                setPowerForMecanumStrafe(-0.2f, heading);
            }

        } else {

            if (isSecondBeacon) {

                setPowerForMecanumStrafe(0.4f, heading);

            } else {

                setPowerForMecanumStrafe(0.2f, heading);
            }
        }
    }

    public void setPowerForMecanumStrafe(float power, int heading) {

        int headingDifference = steadyHeading - heading;

        if (steadyHeading - heading >= 180) {

            headingDifference = steadyHeading - 360 - heading;
        }

        if (heading - steadyHeading >= 180) {

            headingDifference = 360 - heading + steadyHeading;
        }

        if (headingDifference < 0) {

            powerLevels.frontLeftPower = -power; //+ Math.abs(headingDifference / 15);
            powerLevels.backLeftPower = power; //+ Math.abs(headingDifference / 15);
            powerLevels.backRightPower = -power;
            powerLevels.frontRightPower = power;

        } else {

            powerLevels.frontLeftPower = -power;
            powerLevels.backLeftPower = power;
            powerLevels.backRightPower = -power; //+ Math.abs(headingDifference / 15);
            powerLevels.frontRightPower = power; //+ Math.abs(headingDifference / 15);
        }
    }

    public void switchToNextState() {

        elapsedTimeForCurrentState.reset();
        stateIndex++;
        stateStarted = false;

        if (stateIndex >= stateList().length) {

            stateIndex = stateList().length - 1;
        }

        if (stateIndex < 0) {

            stateIndex = 0;
        }

        currentState = stateList()[stateIndex];
        DbgLog.msg("State changed to " + currentState);
    }

    public void switchToNextState(int states) {

        elapsedTimeForCurrentState.reset();
        stateIndex += states;
        stateStarted = false;

        if (stateIndex >= stateList().length) {

            stateIndex = stateList().length - 1;
        }

        if (stateIndex < 0) {

            stateIndex = 0;
        }

        currentState = stateList()[stateIndex];
        DbgLog.msg("State changed to " + currentState);
    }

    public void restartBeaconSequence() {

        elapsedTimeForCurrentState.reset();
        stateIndex -= 2;
        stateStarted = false;

        if (stateIndex >= stateList().length) {

            stateIndex = stateList().length - 1;
        }

        if (stateIndex < 0) {

            stateIndex = 0;
        }

        currentState = stateList()[stateIndex];
        DbgLog.msg("State changed to " + currentState);
    }

    public void startSeg() {

        segment = currentPath[currentPathSegmentIndex];
        elapsedTimeForCurrentSegment.reset();

        int heading = turningGyro.getHeading();
        steadyHeading = heading;

        if (currentPath != null) {

            if (segment.isTurn) {

                segment.Angle += angleOffset;

                if (segment.Angle >= 360) {

                    segment.Angle -= 360;
                }

                runWithoutEncoders();
                double currentAngle = heading;
                segment.isClockwise = !counterclockwiseTurnNeeded(currentAngle);

                if (counterclockwiseTurnNeeded(currentAngle)) {

                    segment.rightPower = -segment.rightPower;

                } else {

                    segment.leftPower = -segment.leftPower;
                }

                powerLevels = new PowerLevels(segment.leftPower, segment.rightPower, segment.leftPower, segment.rightPower);

            } else {

                if (segment.isDelay) {

                    runWithoutEncoders();
                    segment.leftPower = 0.0f;
                    segment.rightPower = 0.0f;

                    powerLevels = new PowerLevels(segment.leftPower, segment.rightPower, segment.leftPower, segment.rightPower);

                } else {

                    if (segment.isHolonomic) {

                        int moveCounts = (int) (segment.LeftSideDistance * countsPerInch);

                        useRunUsingEncoders();
                        addEncoderTarget(moveCounts, moveCounts);

                        if (segment.RightSideDistance < 0.0f) {

                            segment.rightPower *= -1;
                            segment.leftPower *= -1;
                        }

                        setPowerForMecanumStrafe(segment.rightPower, heading);

                    } else {

                        int moveCounts = (int) (segment.LeftSideDistance * countsPerInch);

                        useRunUsingEncoders();
                        addEncoderTarget(moveCounts, moveCounts);

                        if (moveCounts < 0) {

                            segment.leftPower *= -1;
                            segment.rightPower *= -1;
                        }

                        powerLevels = new PowerLevels(segment.leftPower, segment.rightPower, segment.leftPower, segment.rightPower);
                    }
                }
            }

            currentPathSegmentIndex++;
        }
    }

    public void startPath(DrivePathSegment[] path) {

        currentPath = path;
        currentPathSegmentIndex = 0;
        setEncoderTargetsToCurrentPosition();
        useRunUsingEncoders();
        startSeg();
    }

    public boolean pathComplete(int heading) {
        // Wait for this Segment to end and then see what's next.
        if (segmentComplete(heading)) {
            // Start next Segment if there is one.
            if (currentPathSegmentIndex < currentPath.length) {

                TurnOffAllDriveMotors();
                startSeg();

            } else {

                currentPath = null;
                currentPathSegmentIndex = 0;
                TurnOffAllDriveMotors();
                return true;
            }

        } else {

            if (!segment.isTurn) {

                if (segment.isHolonomic) {

                    setPowerForMecanumStrafe(segment.rightPower, turningGyro.getHeading());

                } else {

                    if (segment.isDelay) {

                        TurnOffAllDriveMotors();

                    } else {

                        setPowerForLinearMove(segment.rightPower);
                    }
                }
            }
        }

        return false;
    }

    public int getFrontRightPosition() {

        return frontRightMotor.getCurrentPosition();
    }

    public int getFrontLeftPosition() {

        return frontLeftMotor.getCurrentPosition();
    }

    public int getBackRightPosition() {

        return backRightMotor.getCurrentPosition();
    }

    public int getBackLeftPosition() {

        return backLeftMotor.getCurrentPosition();
    }

    public void setEncoderTargetsToCurrentPosition() {

        currentEncoderTargets.frontLeftTarget = getFrontLeftPosition();
        currentEncoderTargets.frontRightTarget = getFrontRightPosition();
        currentEncoderTargets.backLeftTarget = getBackLeftPosition();
        currentEncoderTargets.backRightTarget = getBackRightPosition();
    }

    public boolean segmentComplete(int heading) {

        if (segment.isTurn) {

            if (turnComplete()) {

                return true;

            } else {

                int headingDifference = (int) segment.Angle - heading;

                if ((int) segment.Angle - heading >= 180) {

                    headingDifference = (int) segment.Angle - 360 - heading;
                }

                if (heading - (int) segment.Angle >= 180) {

                    headingDifference = 360 - heading + (int) segment.Angle;
                }

                if (segment.isClockwise) {

                    if (headingDifference < 0) {

                        segment.leftPower *= -1;
                        segment.rightPower *= -1;
                        segment.isClockwise = false;
                    }

                } else {

                    if (headingDifference > 0) {

                        segment.leftPower *= -1;
                        segment.rightPower *= -1;
                        segment.isClockwise = true;
                    }
                }

                powerLevels = new PowerLevels(segment.leftPower, segment.rightPower, segment.leftPower, segment.rightPower);

                telemetry.addData("HD" , headingDifference);
                telemetry.addData("LHD", lastHeadingDifference);

                return false;
            }

        } else {

            if (segment.isDelay) {

                return delayComplete();

            } else {

                return linearMoveComplete();
            }
        }
    }

    public boolean turnComplete() {

        int heading = turningGyro.getHeading();

        return Math.abs(segment.Angle) <= heading + TURNING_ANGLE_MARGIN &&
                Math.abs(segment.Angle) >= heading - TURNING_ANGLE_MARGIN;
    }

    public boolean delayComplete() {

        return elapsedTimeForCurrentSegment.time() >= segment.delayTime;
    }

    public boolean linearMoveComplete() {

        int frontLeftTarget = currentEncoderTargets.frontLeftTarget;
        int frontRightTarget = currentEncoderTargets.frontRightTarget;
        int backLeftTarget = currentEncoderTargets.backLeftTarget;
        int backRightTarget = currentEncoderTargets.backRightTarget;

        return (//isPositionClose(getFrontRightPosition(), frontRightTarget) ||
                //isPositionClose(getFrontLeftPosition(), frontLeftTarget) ||
                isPositionClose(getBackRightPosition(), backRightTarget) ||
                isPositionClose(getBackLeftPosition(), backLeftTarget)) ||
                (//isPastTarget(getFrontRightPosition(), frontRightTarget, segment.RightSideDistance) ||
                        //isPastTarget(getFrontLeftPosition(), frontLeftTarget, segment.LeftSideDistance) ||
                        isPastTarget(getBackRightPosition(), backRightTarget, segment.RightSideDistance) ||
                        isPastTarget(getBackLeftPosition(), backLeftTarget, segment.LeftSideDistance));
    }

    public boolean isPositionClose(int position, int target) {

        return Math.abs(target - position) < ENCODER_TARGET_MARGIN;
    }

    public boolean isPastTarget(int position, int target, float distanceToMove) {

        if (distanceToMove < 0) {

            return position < target;
        }

        return position > target;
    }

    public void pushBeaconButton(int leftSensorRead) {

        telemetry.addData("Left Sensor", leftSensorRead);

        if (leftSensorRead >= 2) {

            rightBeaconServoIn();
            leftBeaconServoOut();

        } else {

            rightBeaconServoOut();
            leftBeaconServoIn();
        }
    }

    public void addEncoderTarget(int leftEncoderAdder, int rightEncoderAdder) {

        currentEncoderTargets.frontLeftTarget += leftEncoderAdder;
        currentEncoderTargets.frontRightTarget += rightEncoderAdder;
        currentEncoderTargets.backLeftTarget += leftEncoderAdder;
        currentEncoderTargets.backRightTarget += rightEncoderAdder;
    }

    public boolean counterclockwiseTurnNeeded(double currentAngle) {

        telemetry.addData("Angle: ", currentAngle);

        if (currentAngle < Math.abs(segment.Angle)) {

            return (Math.abs(segment.Angle) - currentAngle) >= 180.0f;
        }

        return (currentAngle - Math.abs(segment.Angle)) <= 180.0f;
    }

    public void useRunUsingEncoders() {

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void throwBall(ElapsedTime ElapsedThrowingTime, float throwingTime) {

        if (ElapsedThrowingTime.time() >= throwingTime) {

            lowerAutoThrowingArm(ElapsedThrowingTime, throwingTime);

        } else {

            throwingArmPowerLevel = throwingPower();
            //throwingArmPowerLevel = 0.8f; //for when gearbox starts wearing down
        }
    }

    public void lowerAutoThrowingArm(ElapsedTime ElapsedThrowingTime, float throwingTime) {

        if (ElapsedThrowingTime.time() >= throwingTime * 1.5) {

            throwingArmPowerLevel = 0.0f;

        } else {

            throwingArmPowerLevel = -0.8f;
        }
    }

    public abstract boolean isRed();

    public abstract State[] stateList();

    public abstract float throwingPower();
}