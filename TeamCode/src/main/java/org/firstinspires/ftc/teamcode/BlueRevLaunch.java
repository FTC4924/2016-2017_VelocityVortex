package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/6/2017.
 */
@Autonomous(name = "BlueRevLaunch", group = "Revolution")
public class BlueRevLaunch extends RevCornerBase {

    public BlueRevLaunch() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(10.0f),
                new DrivePathSegment(2.00f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(36.0f, 0.5f, DrivePathSegment.TURN),
                new DrivePathSegment(14.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(55.0f, 0.5f, DrivePathSegment.TURN),
                new DrivePathSegment(1.0f),


        };

        postThrowingPath = new DrivePathSegment[] {

                new DrivePathSegment(-5.0f, 0.8f, DrivePathSegment.LINEAR),
        };

        stop = new DrivePathSegment [] {

                new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
        };
    }

    public boolean isRed() {

        return false;
    }

    public float throwingPower() {

        return 0.9f;
    }
}

