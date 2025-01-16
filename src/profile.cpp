#include "main.h"

MotionProfile::MotionProfile(CubicHermiteSpline* path, double maxSpeed) {
    // assigns the passed-in values to instance variables
    this->path = path;
    this->maxSpeed = maxSpeed;
    // default velocity zoning
    // (0-10% goes from x0.1-1, 10-90% remains at x1, 90-100% goes from x1-0)
    this->zones = {
        {0, 0.1, {9, 0.1}},
        {0.1, 0.9, {0, 1}},
        {0.9, 1, {-10, 10}}
    };
    // ends by creating the profile
    this->createProfile();
}

MotionProfile::MotionProfile(CubicHermiteSpline* path, std::vector<std::vector<Point>> zonePoints, double maxSpeed) {
    // assigns the passed-in values to instance variables
    this->path = path;
    this->maxSpeed = maxSpeed;
    // custom velocity zoning
    this->constructWithCustomZones(zonePoints);
    // ends by creating the profile
    this->createProfile();
}


void MotionProfile::constructWithCustomZones(std::vector<std::vector<Point>> zoneLinePoints) {
    this->zones = {};
    Line currentZoneLine;
    // generates the velocity lines
    for (int i = 0; i < zoneLinePoints.size(); i++) {
        currentZoneLine = findLineWithPoints(zoneLinePoints[i][0], zoneLinePoints[i][1]);
        this->zones.push_back({zoneLinePoints[i][0].x, zoneLinePoints[i][1].x, currentZoneLine});
    }
    // tests to make sure each point has a valid velocity, with a sample size of 1000
    for (int i = 0; i < 1000; i++) {

        double t = i / (double) 1000;
        Zone assignedZone = {NAN, NAN, {NAN, NAN}};

        for (int j = 0; j < this->zones.size(); j++) {
            if ((this->zones[j].startT <= t) && (this->zones[j].endT >= t)) {
                assignedZone = this->zones[j];
            }
        }
        // if a point doesn't have a valid zone, the default zoning is used instead
        if (std::isnan(assignedZone.startT)) {
            pros::lcd::print(0, "Invalid velocity zoning! Using default as substitute.");
            this->zones = {
                {0, 0.1, {9, 0.1}},
                {0.1, 0.9, {0, 1}},
                {0.9, 1, {-10, 10}}
            };
            return;
        }
    }
    return;
}

void MotionProfile::generateVelocities(std::vector<UltraPose> pointList) {
    for (int i = 0; i < pointList.size(); i++) {

        double t = i / (double) pointList.size();
        Zone assignedZone = {NAN, NAN, {NAN, NAN}};

        // iterates through the zones until the one that contains the current point's t value is found
        for (int j = 0; j < this->zones.size(); j++) {
            if ((this->zones[j].startT <= t) && (this->zones[j].endT >= t)) {
                assignedZone = this->zones[j];
            }
        }

        // velocity calculations
        // the zone line is solved for at the point t to find the velocity multiplier
        double linearVelocityMultiplier = (assignedZone.zoneLine.slope * t) + assignedZone.zoneLine.yIntercept;
        // the linear velocity is simply the multiplier (which is a percentage) of the maximum speed allowed by the profile
        double linearVelocity = linearVelocityMultiplier * this->maxSpeed;
        // the angular velocity is the curvature of the current point multiplied by the current linear velocity
        double angularVelocity = linearVelocity * pointList[i].curvature;

        
        profile.push_back({pointList[i].x, pointList[i].y, pointList[i].heading, linearVelocity, angularVelocity, t});
    }
}

void MotionProfile::createProfile(void) {
    // first, creates the profile with 1000 points as a baseline
    double numPoints = 1000;
    double step = 1 / numPoints;
    std::vector<UltraPose> pointList = this->path->entirePath(numPoints);
    this->generateVelocities(pointList);

    // second, calculates the total time and length of the profile
    double totalTime = 0;
    double totalLength = 0;
    double segLength = 0;
    for (int i = 0; i < pointList.size() - 1; i += 1) {
        segLength = calculateDistance({this->profile[i].x, this->profile[i].y}, {this->profile[i+1].x, this->profile[i+1].y});
        totalTime += segLength / this->profile[i].linVel;
        totalLength += segLength;
    }

    // third and finally, recalculates the profile with a variable number of points, equally spaced out at intervals of 5 ms
    int trueNumPoints = std::round((totalTime * 1000) / 5);
    pointList.clear();
    profile.clear();
    pointList = this->path->entirePath(trueNumPoints);
    this->generateVelocities(pointList);
}

// given a value of t on the profile, finds the profile's closest generated point
MPPoint MotionProfile::findNearestPoint(double givenT) {
    MPPoint closestCandidate;
    double closestDifference = 10000;
    for (int i = 0; i < profile.size(); i++) {
        if (std::abs(givenT - this->profile[i].t) < closestDifference) {
            closestCandidate = this->profile[i];
            closestDifference = std::abs(givenT - this->profile[i].t);
        }
    }
    return closestCandidate;
}

Direction MotionProfile::findCurveDirectionOfPoint(MPPoint currentPoint) {
    MPPoint prevPoint = this->profile[(currentPoint.t * this->profile.size()) - 1];
    Direction dir;

    if (prevPoint.heading > currentPoint.heading) {dir = RIGHT;}
    else if (currentPoint.heading > prevPoint.heading) {dir = LEFT;}
    else {dir = STRAIGHT;}

    return dir;
}