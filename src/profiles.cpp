#include "main.h"


std::vector<MotionProfile*> RedAWPSetup() {
    // spline setup
    // CubicHermiteSpline goalSpline = CubicHermiteSpline({-54.75, 13.25}, {28.7, 45}, {35.14, 29.54}, {11.6, 17.9});
    CubicHermiteSpline goalSpline = CubicHermiteSpline({-54.75, 13.25}, {10, 36}, {-20, 25}, {10, 36});
    CubicHermiteSpline innerRingSpline = CubicHermiteSpline({-20, 25}, {-25, 51}, {-23, 44}, {-29, 83});
    CubicHermiteSpline outerRingSpline = CubicHermiteSpline({-23, 44}, {-34, 77}, {-50, 39}, {-49, 20});
    CubicHermiteSpline crossSpline = CubicHermiteSpline({-49.5, 39}, {-50, 20}, {-50, -12}, {-43, -41.5});
    CubicHermiteSpline southernRingSpline = CubicHermiteSpline({-50, -12}, {-38, -125.5}, {-22, -56}, {-13.5, -83});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({-26, -56}, {-24.5, -22.5}, {-32, -23}, {-41, 3});
    CubicHermiteSpline ladder2Spline = CubicHermiteSpline({-32, -23}, {-41, 3.5}, {-11, -15}, {-2, -13});

    // profile setup
    MotionProfile* goalProfile = new MotionProfile(&goalSpline, RPMtoIPS(600));
    MotionProfile* innerRingProfile = new MotionProfile(&innerRingSpline, RPMtoIPS(600), 
        {
            {{0, 0.1}, {0.05, 1}}, 
            {{0.05, 1}, {1, 1}}
        }
    );
    MotionProfile* outerRingProfile = new MotionProfile(&outerRingSpline, RPMtoIPS(600),
        {
            {{0, 0.1}, {0.1, 1}}, 
            {{0.1, 1}, {1, 1}}
        }
    );
    MotionProfile* crossProfile = new MotionProfile(&crossSpline, RPMtoIPS(600),
        {
            {{0, 1}, {0.9, 1}},
            {{0.5, 1}, {0.7, 0.4}},
            {{0.7, 0.4}, {1, 0.4}}
        }
    );
    MotionProfile* southernRingProfile = new MotionProfile(&southernRingSpline, RPMtoIPS(600), {
        {
            {{0, 0.4}, {0.4, 0.4}},
            {{0.4, 0.4}, {0.5, 1}},
            {{0.5, 1}, {0.9, 1}},
            {{0.9, 1}, {1, 0}}
        }
    });
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, RPMtoIPS(600));
    MotionProfile* ladder2Profile = new MotionProfile(&ladder2Spline, RPMtoIPS(600));

    return {goalProfile, innerRingProfile, outerRingProfile, crossProfile, southernRingProfile, ladderProfile, ladder2Profile};
}

std::vector<MotionProfile*> BlueAWPSetup() {
    // spline setup
    // CubicHermiteSpline goalSpline = CubicHermiteSpline({-54.75, 13.25}, {28.7, 45}, {35.14, 29.54}, {11.6, 17.9});
    CubicHermiteSpline goalSpline = CubicHermiteSpline({54.75, 13.25}, {-10, 36}, {20, 25}, {-10, 36});
    CubicHermiteSpline innerRingSpline = CubicHermiteSpline({20, 25}, {22, 29}, {33, 58.5}, {90.5, 73.5});
    CubicHermiteSpline outerRingSpline = CubicHermiteSpline({0, 0}, {0, 0}, {0, 0}, {0, 0});
    CubicHermiteSpline crossSpline = CubicHermiteSpline({33, 58.5}, {82.5, 56}, {53, -12}, {49, -43.5});
    CubicHermiteSpline southernRingSpline = CubicHermiteSpline({53, -12}, {38, -125.5}, {26, -56}, {13.5, -83});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({26, -56}, {24.5, -22.5}, {32, -23}, {41, 3});
    CubicHermiteSpline ladder2Spline = CubicHermiteSpline({32, -23}, {41, 3.5}, {11, -15}, {2, -13});

    // profile setup
    MotionProfile* goalProfile = new MotionProfile(&goalSpline, RPMtoIPS(600));
    MotionProfile* innerRingProfile = new MotionProfile(&innerRingSpline, RPMtoIPS(600), 
        {
            {{0, 0.1}, {0.05, 1}}, 
            {{0.05, 1}, {1, 1}}
        }
    );
    MotionProfile* outerRingProfile = new MotionProfile(&outerRingSpline, RPMtoIPS(600)

    );
    MotionProfile* crossProfile = new MotionProfile(&crossSpline, RPMtoIPS(600),
        {
            {{0, 0}, {0.1, 1}},
            {{0.1, 1}, {0.55, 1}},
            {{0.55, 1}, {0.7, 0.4}},
            {{0.7, 0.4}, {1, 0.4}}
        }
    );
    MotionProfile* southernRingProfile = new MotionProfile(&southernRingSpline, RPMtoIPS(600), {
        {
            {{0, 0.4}, {0.3, 0.4}},
            {{0.3, 0.4}, {0.4, 1}},
            {{0.5, 1}, {0.9, 1}},
            {{0.9, 1}, {1, 0}}
        }
    });
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, RPMtoIPS(600));
    MotionProfile* ladder2Profile = new MotionProfile(&ladder2Spline, RPMtoIPS(600));

    return {goalProfile, innerRingProfile, outerRingProfile, crossProfile, southernRingProfile, ladderProfile, ladder2Profile};
}

std::vector<MotionProfile*> RedGoalRushSetup() {
    // spline setup
    CubicHermiteSpline rushSpline = CubicHermiteSpline({50.5, -35.5}, {22.4, -36.5}, {-16.8, -48}, {22.37, -36.3});
    CubicHermiteSpline secondGoalSpline = CubicHermiteSpline({-26.98, -53.06}, {-95.7, -25.5}, {-21.83, -21.96}, {2.6, -10.6});
    CubicHermiteSpline cornerSpline = CubicHermiteSpline({-21.9, -22.03}, {-73, -55.3}, {-62.2, -52.4}, {-69.1, -84.4});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({-62.14, -52.46}, {80.6, -87}, {-9.1, -28.1}, {7.8, 3.4});
    // profile setup
    MotionProfile* rushProfile = new MotionProfile(&rushSpline, 600);
    MotionProfile* secondGoalProfile = new MotionProfile(&secondGoalSpline, 400);
    MotionProfile* cornerProfile = new MotionProfile(&cornerSpline, 600);
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, 500);

    return {rushProfile, secondGoalProfile, cornerProfile, ladderProfile};
}

std::vector<MotionProfile*> BlueGoalRushSetup() {
    // spline setup
    CubicHermiteSpline rushSpline = CubicHermiteSpline({50.5, -35.5}, {-17.2, -54.1}, {13.5, -44.7}, {-17.2, -54.2});
    CubicHermiteSpline secondGoalSpline = CubicHermiteSpline({27.03, -42}, {75.8, -34.54}, {21.84, -22.2}, {-2.6, -10.6});
    CubicHermiteSpline cornerSpline = CubicHermiteSpline({21.9, -22.03}, {73, -55.3}, {53.3, -62.2}, {82.1, -72.3});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({53.4, -62.3}, {117.8, 45.8}, {9.1, -28.1}, {17.4, -8.6});
    // profile setup
    MotionProfile* rushProfile = new MotionProfile(&rushSpline, RPMtoIPS(1000), {
        {
            {{0, 1}, {1, 1}}
        }
    });
    MotionProfile* secondGoalProfile = new MotionProfile(&secondGoalSpline, RPMtoIPS(400));
    MotionProfile* cornerProfile = new MotionProfile(&cornerSpline, RPMtoIPS(600));
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, RPMtoIPS(500));

    return {rushProfile, secondGoalProfile, cornerProfile, ladderProfile};
}