#include "main.h"

MotionProfile::MotionProfile(std::vector<UltraPose> pointList, double maxSpeed) {
    this->pointList = pointList;
    this->maxSpeed = maxSpeed;
    this->zones = {
        {0, 0.1, {9, 0.1}},
        {0.1, 0.9, {0, 1}},
        {0.9, 1, {-10, 10}}
    };
    this->generateVelocities();
}

MotionProfile::MotionProfile(std::vector<UltraPose> pointList, std::vector<std::vector<Point>> zonePoints, double maxSpeed) {
    this->pointList = pointList;
    this->maxSpeed = maxSpeed;
    this->constructWithCustomZones(zonePoints);
    this->generateVelocities();
}


void MotionProfile::constructWithCustomZones(std::vector<std::vector<Point>> zoneLinePoints) {
    this->zones = {};
    Line currentZoneLine;
    for (int i = 0; i < zoneLinePoints.size(); i++) {
        currentZoneLine = findLineWithPoints(zoneLinePoints[i][0], zoneLinePoints[i][1]);
        this->zones.push_back({zoneLinePoints[i][0].x, zoneLinePoints[i][1].x, currentZoneLine});
    }
    for (int i = 0; i < pointList.size(); i++) {
        double t = i / (double) pointList.size();
        Zone assignedZone = {NAN, NAN, {NAN, NAN}};

        for (int j = 0; j < this->zones.size(); j++) {
            if ((this->zones[j].startT <= t) && (this->zones[j].endT >= t)) {
                assignedZone = this->zones[j];
            }
        }
        if (std::isnan(assignedZone.startT)) {
            throw std::runtime_error("Wow, your velocity zoning is like your attention span — full of gaps where there should be continuity.");
        }
    }
}

void MotionProfile::generateVelocities(void) {

    for (int i = 0; i < pointList.size(); i++) {

        double t = i / (double) pointList.size();
        Zone assignedZone = {NAN, NAN, {NAN, NAN}};

        for (int j = 0; j < this->zones.size(); j++) {
            if ((this->zones[j].startT <= t) && (this->zones[j].endT >= t)) {
                assignedZone = this->zones[j];
                break;
            }
        }
        if (std::isnan(assignedZone.startT)) {
            throw std::runtime_error("Wow, your velocity zoning is like your attention span — full of gaps where there should be continuity.");
        }
        double linearVelocityMultiplier = (assignedZone.zoneLine.slope * t) + assignedZone.zoneLine.yIntercept;
        double linearVelocity = linearVelocityMultiplier * this->maxSpeed;
        double angularVelocity = linearVelocity * pointList[i].curvature;

        
        profile.push_back({pointList[i].x, pointList[i].y, pointList[i].heading, linearVelocity, angularVelocity, t});
    }

    double segLength = 0;
    this->totalTime = 0;
    for (int i = 0; i < profile.size() - 1; i++) {
        segLength = calculateDistance({this->profile[i].x, this->profile[i].y}, {this->profile[i+1].x, this->profile[i+1].y});
        totalTime += segLength / this->profile[i].linVel;
    }

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