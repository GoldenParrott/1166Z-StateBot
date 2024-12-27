#include "profiling.h"

double findHeadingOfLine(
    Point point1, // the initial point
    Point point2 // the final point
)
{
    // finds the difference between the x- and y- values to get the rise and run of the line
    double xChange = point2.x - point1.x;
    double yChange = point2.y - point1.y;

    bool yIsPositive = yChange > 0;
    bool xIsPositive = xChange > 0;

    // finds the positive position of the angle in relation to the previous multiple of 90 degrees
    double triangleAngle = 0;
    if ((yIsPositive && xIsPositive) || (!yIsPositive && !xIsPositive)) { // quadrants 1 or 3
        triangleAngle = std::atan(std::abs(xChange) / std::abs(yChange)); // tangent is opposite/adjacent, and x is opposite in these cases
    }
    else { // quadrants 2 or 4
        triangleAngle = std::atan(std::abs(yChange) / std::abs(xChange)); // tangent is opposite/adjacent, and y is opposite in these cases
    }
    triangleAngle = (triangleAngle * 180) / 3.14;

    double heading = 0;
    if (xIsPositive && yIsPositive) { // quadrant 1
       heading = triangleAngle;
    } else if (xIsPositive && !yIsPositive) { // quadrant 2
        heading = triangleAngle + 90;
    } else if (!xIsPositive && !yIsPositive) { // quadrant 3
        heading = triangleAngle + 180;
    } else if (!xIsPositive && yIsPositive) { // quadrant 4
        heading = triangleAngle + 270;
    }

    return heading;
}

double calculateDistance(Point point1, Point point2) {
    return std::sqrt(std::pow((point2.x - point1.x), 2) + std::pow((point2.y - point1.y), 2));
}

Line findLineWithPoints(
    Point point1,
    Point point2
)
{
    Line Line1;
    Line LinePerp;

    if (point2.y - point1.y == 0) {
        Line1.slope = 0;
        Line1.yIntercept = point1.x;
        return Line1;
    }
    else if (point2.x - point1.x == 0) {
        Line1.slope = NAN;
        Line1.yIntercept = point2.y;
        return LinePerp;
    }
    
    // handles all y = mx + b cases
    Line1.slope = (point2.y - point1.y) / (point2.x - point1.x); // calculates the slope of the first line
    Line1.yIntercept = (-1 * (Line1.slope * point1.x)) + point1.y; // formula for y-intercept based on point-slope form
    return Line1;
}

QuadraticPolyData derivativeOfCubicPoly(CubicPolyData cubicPoly) {
    // the power rule is used on each term to find the derivative of the 
    double a = cubicPoly.a * 3; // power rule to find the x^2 coefficient
    double b = cubicPoly.b * 2; // power rule to find the x^1 coefficient
    double c = cubicPoly.c; // power rule to find the x^0 coefficient

    QuadraticPolyData cubicDerivative = {a, b, c};

    return cubicDerivative;
}

Line derivativeOfQuadratic(QuadraticPolyData quadPoly) {
    // the power rule is used on each term to find the derivative of the 
    double a = quadPoly.a * 2; // power rule to find the x^1 coefficient
    double b = quadPoly.b; // power rule to find the x^0 coefficient

    Line quadDerivative = {a, b};

    return quadDerivative;
}

Line findLineWithHeading(
    Point point1, // a point on the line
    int heading // inertial heading
)
{
    // calculates the angle of only the triangle by subtracting from it based on its quadrant
    double triangleAngle = 0;
    if (heading < 90) {
       triangleAngle = heading;
    } else if (heading < 180) {
        triangleAngle = heading - 90;
    } else if (heading < 270) {
        triangleAngle = heading - 180;
    } else if (heading < 360) {
        triangleAngle = heading - 270;
    }
    // treats the distance moved as the hypotenuse of and the heading as the base angle of a triangle
    // and uses them to calculate the value of both legs (the changes in x and y)
    double xChange = 0;
    double yChange = 0;
    double hypotenuse = 1; // because the hypotenuse is an infinite line, the length for this calculation does not matter, so I picked 1 for ease of calculation

    // if the heading is in quadrants 1 or 3, then the x-value is the opposite leg (sine) and the y-value is the adjacent leg (cosine)
    if ((heading < 90) || (heading >= 180 && heading < 270)) {
        xChange = std::sin(((triangleAngle * 3.141592) / 180)) * hypotenuse;
        yChange = std::cos(((triangleAngle * 3.141592) / 180)) * hypotenuse;
    }
    // otherwise, if the heading is in quadrants 2 or 4, then the x-value is the adjacent leg (cosine) and the y-value is the opposite leg (sine)
    else {
        xChange = std::cos(((triangleAngle * 3.141592) / 180)) * hypotenuse;
        yChange = std::sin(((triangleAngle * 3.141592) / 180)) * hypotenuse;
    }

    // slope of line = rise / run
    double slope = yChange / xChange;

    // calculates the y-intercept with a derived form of the point-slope form of an equation
    double yIntercept = (-1 * (slope * point1.x)) + point1.y;

    Line line = {slope, yIntercept};

    return line;
}

Line calculatePerpendicular(
    Point point1, // has a line between it and point2 that will have a line perpendicular to it
    Point point2 // the point that the perpendicular line will cross
)
{
    Line Line1;
    Line LinePerp;

    if (point2.y - point1.y == 0) { // handles the case if y does not change (line takes the form of x = k)
        LinePerp.slope = NAN;
        LinePerp.yIntercept = point2.x; // yIntercept serves as the value of k (in x = k) for this case
        return LinePerp;
    }
    else if (point2.x - point1.x == 0) { // handles the case if x does not change (line takes the form of y = k)
        LinePerp.slope = 0;
        LinePerp.yIntercept = point2.y; // yIntercept serves as the value of k (in y = k) for this case
        return LinePerp;
    }
    
    // handles all y = mx + b cases
    Line1.slope = (point2.y - point1.y) / (point2.x - point1.x); // calculates the slope of the first line
    Line1.yIntercept = (-1 * (Line1.slope * point1.x)) + point1.y; // formula for y-intercept based on point-slope form
    

    LinePerp.slope = -1 * (1 / Line1.slope); // slope of a perpendicular line is the negative reciprocal of the slope of the original
    LinePerp.yIntercept = (-1 * (LinePerp.slope * point2.x)) + point2.y;

    return LinePerp;
}