#include <var.h>
#include <Arduino.h>

enum SegmentType
{
    SEG_FORWARD,
    SEG_BACKWARD,
    SEG_ROTATE_R,
    SEG_ROTATE_L,
    SEG_STOP
};

struct TrackSegment
{
    SegmentType type;
    float value;
};

#define MAX_TRACK_SEGMENTS 64

TrackSegment track[MAX_TRACK_SEGMENTS];

int trackLen = 0;
int currentSeg = 0;

bool autoRunning = false;
bool autoPaused = false;

float segStartDistance = 0;
float segStartHeading = 0;

bool segInitialized = false;

float headingDiff(float from, float to)
{
    float d = to - from;

    while(d > 180)
        d -= 360;

    while(d < -180)
        d += 360;

    return d;
}

bool autonomousStartTrack()
{
    if(trackLen == 0)
        return false;

    currentSeg = 0;
    segInitialized = false;

    autoRunning = true;
    autoPaused = false;

    return true;
}

void autonomousAbort()
{
    autoRunning = false;
    autoPaused = false;

    currentSeg = 0;
    segInitialized = false;

    data = 0;
}

bool autonomousIsRunning()
{
    return autoRunning;
}

bool autonomousIsPaused()
{
    return autoPaused;
}

void clearTrack()
{
    trackLen = 0;
}

bool addSegment(int type, float value)
{
    if(trackLen >= MAX_TRACK_SEGMENTS)
        return false;

    track[trackLen].type = (SegmentType)type;
    track[trackLen].value = value;

    trackLen++;

    return true;
}

bool loadTrack(String trackString)
{
    clearTrack();

    while(trackString.length() > 0)
    {
        int comma = trackString.indexOf(',');

        String token;

        if(comma == -1)
        {
            token = trackString;
            trackString = "";
        }
        else
        {
            token = trackString.substring(0, comma);
            trackString = trackString.substring(comma + 1);
        }

        token.trim();

        if(token.length() == 0)
            continue;

        char cmd = toupper(token[0]);

        switch(cmd)
        {
            case 'F':
                addSegment(SEG_FORWARD,
                        token.substring(1).toFloat());
                break;

            case 'B':
                addSegment(SEG_BACKWARD,
                        token.substring(1).toFloat());
                break;

            case 'R':
                addSegment(SEG_ROTATE_R,
                        token.substring(1).toFloat());
                break;

            case 'L':
                addSegment(SEG_ROTATE_L,
                        token.substring(1).toFloat());
                break;

            case 'S':
                addSegment(SEG_STOP, 0);
                break;

            default:
                Serial.print("Invalid token: ");
                Serial.println(token);
                return false;
        }
    }

    return autonomousStartTrack();
}

void autonomousUpdate()
{
    if(!autoRunning)
        return;

    if(autoPaused)
    {
        data = 0;
        return;
    }

    if(currentSeg >= trackLen)
    {
        autoRunning = false;
        data = 0;
        return;
    }

    TrackSegment &seg = track[currentSeg];

    if(!segInitialized)
    {
        segStartDistance = getDistance();
        segStartHeading = heading;

        segInitialized = true;
    }

    switch(seg.type)
    {
        case SEG_FORWARD:
        {
            float travelled = getDistance() - segStartDistance;

                if(travelled >= seg.value) {
                    data = 0;
                    currentSeg++;
                    segInitialized = false;
                }
                else {
                    data = 1;
                }
                break;

            
        }

        case SEG_BACKWARD:
        {
            float travelled = segStartDistance - getDistance();

            if(travelled >= seg.value)
            {
                data = 0;
                currentSeg++;
                segInitialized = false;
            }
            else
            {
                data = 2;
            }

            break;
        }

        case SEG_ROTATE_L:
        {
            float turned = -headingDiff(segStartHeading, heading);

            if(turned >= seg.value)
            {
                data = 0;

                currentSeg++;
                segInitialized = false;
            }
            else
            {
                data = 21;
            }

            break;
        }

        case SEG_ROTATE_R:
        {
            float turned = headingDiff(segStartHeading, heading);

            if(turned >= seg.value)
            {
                data = 0;

                currentSeg++;
                segInitialized = false;
            }
            else
            {
                data = 11;
            }

            break;
        }


        case SEG_STOP:
        {
            data = 0;
            autoRunning = false;
            break;
        }
    }
}