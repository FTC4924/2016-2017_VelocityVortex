package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 2/18/2017.
 */

public abstract class RevLaunchAndRampBase extends RevolutionAutonomousBase{
    @Override
    public State[] stateList() {

        return new State[] {

                State.STATE_INITIAL,
                State.STATE_START_LAUNCH_PATH,
                State.STATE_POSITION_FOR_BALL,
                State.STATE_LAUNCH_BALL,
                State.STATE_LOAD_BALL,
                State.STATE_WAIT_FOR_BALL,
                State.STATE_LAUNCH_BALL,
                State.STATE_START_BEACON_PATH,
                State.STATE_DRIVE,
                State.STATE_STOP
        };
    }
}
