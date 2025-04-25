#include "main.h"


std::vector<MotionProfile*> AWPSetup(int color) {
    // spline setup
    // CubicHermiteSpline goalSpline = CubicHermiteSpline({-54.75, 13.25}, {28.7, 45}, {35.14, 29.54}, {11.6, 17.9});
    CubicHermiteSpline goalSpline = CubicHermiteSpline({color * 54.75, 13.25}, {color * -10, 36}, {color * 20, 25}, {color * -10, 36});
    CubicHermiteSpline innerRingSpline = CubicHermiteSpline({color * 20, 25}, {color * 25, 51}, {color * 23, 44}, {color * 29, 83});
    CubicHermiteSpline outerRingSpline = CubicHermiteSpline({color * 23, 44}, {color * 34, 77}, {color * 50, 39}, {color * 49, 20});
    CubicHermiteSpline crossSpline = CubicHermiteSpline({color * 49.5, 39}, {color * 50, 20}, {color * 50, -12}, {color * 43, -41.5});
    CubicHermiteSpline southernRingSpline = CubicHermiteSpline({color * 50, -12}, {color * 38, -125.5}, {color * 22, -56}, {color * 13.5, -83});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({color * 26, -56}, {color * 24.5, -22.5}, {color * 32, -23}, {color * 41, 3});
    CubicHermiteSpline ladder2Spline = CubicHermiteSpline({color * 32, -23}, {color * 41, 3.5}, {color * 11, -15}, {color * 2, -13});
    CubicHermiteSpline newLadderSpline = CubicHermiteSpline({color * 23, 44}, {color * 28, 9.3}, {color * 15, 11}, {color * -1.5, -10.3});

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
    MotionProfile* newLadderProfile = new MotionProfile(&newLadderSpline, RPMtoIPS(600));

    return {goalProfile, innerRingProfile, outerRingProfile, crossProfile, southernRingProfile, ladderProfile, ladder2Profile, newLadderProfile};
}

std::vector<MotionProfile*> GoalRushSetup(int color) {
    // spline setup
    CubicHermiteSpline rushSpline = CubicHermiteSpline({color * 53, -38.75}, {color * -8.66, -54.4}, {color * 13.9, -47}, {color * -9.1, -54.8});
    CubicHermiteSpline secondGoalSpline = CubicHermiteSpline({color * 24.5, -47}, {color * 28.7, -26.3}, {color * 21.91, -12}, {color * -8.4, 72});
    CubicHermiteSpline cornerSpline = CubicHermiteSpline({color * 22, -10}, {color * 39, -63}, {color * 51.7, -63.2}, {color * 81.4, -76});
    CubicHermiteSpline cornerRingSpline = CubicHermiteSpline({color * 51, -63.5}, {color * 98, -8.5}, {color * 55, -10.3}, {color * 42, 14});
    CubicHermiteSpline fetchSpline = CubicHermiteSpline({color * 55.34, -10.4}, {color * 91, -77.3}, {color * 12.2, -47.2}, {color * -4, -56.7});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({color * 12, -47.25}, {color * 59, -35.3}, {color * 13.3, -13.8}, {color * -13.5, 21.7});
    // profile setup
    MotionProfile* rushProfile = new MotionProfile(&rushSpline, RPMtoIPS(600),
        {
            {{0, 0.1}, {0.05, 1}}, 
            {{0.05, 1}, {1, 1}}
        }
    );
    MotionProfile* secondGoalProfile = new MotionProfile(&secondGoalSpline, RPMtoIPS(400));
    MotionProfile* cornerProfile = new MotionProfile(&cornerSpline, RPMtoIPS(600));
    MotionProfile* cornerRingProfile = new MotionProfile(&cornerRingSpline, RPMtoIPS(600));
    MotionProfile* fetchProfile = new MotionProfile(&cornerRingSpline, RPMtoIPS(600));
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, RPMtoIPS(500),
        {
            {{0, 0.1}, {0.1, 1}},
            {{0.1, 1}, {0.4, 0.5}},
            {{0.4, 0.5}, {1, 0.5}}
        }
    );

    return {rushProfile, secondGoalProfile, cornerProfile, cornerRingProfile, fetchProfile, ladderProfile};
}

std::vector<MotionProfile*> RingSetup(int color) {
    // spline setup
    CubicHermiteSpline centerSpline = CubicHermiteSpline({color * 50, 46.75}, {color * -80.5, -30}, {color * 6, 59}, {color * 5, 135});
    CubicHermiteSpline goalSpline = CubicHermiteSpline({color * 6, 59}, {color * 9, -10}, {color * 24, 22.75}, {color * 78.5, -17});
    CubicHermiteSpline cornerSpline = CubicHermiteSpline({color * 24, 22.75}, {color * -37, 61.2}, {color * 52, 59}, {color * 90, 82.75});
    CubicHermiteSpline sweepSpline = CubicHermiteSpline({color * 52, 59}, {color * 85.75, -39}, {color * 42.5, -9}, {color * 29.5, -55.25});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({color * 42.5, -9}, {color * 65, 56.75}, {color * 13.5, 11.5}, {color * -9, -58});
    // profile setup
    MotionProfile* centerProfile = new MotionProfile(&centerSpline, RPMtoIPS(600));
    MotionProfile* goalProfile = new MotionProfile(&goalSpline, RPMtoIPS(600));
    MotionProfile* cornerProfile = new MotionProfile(&cornerSpline, RPMtoIPS(600));
    MotionProfile* sweepProfile = new MotionProfile(&sweepSpline, RPMtoIPS(600));
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, RPMtoIPS(600));

    return {centerProfile, goalProfile, cornerProfile, sweepProfile, ladderProfile};
}

std::vector<MotionProfile*> SkillsSetup() {
    // spline setup
    CubicHermiteSpline goalQ3Spline = CubicHermiteSpline({-50, 0}, {-50, -35}, {-50, -28.34}, {-50, -35});
    CubicHermiteSpline ringMidQ3Spline = CubicHermiteSpline({-50, -28}, {-14, -23}, {-22, -18}, {-14, -23});
    CubicHermiteSpline wallQ3Spline = CubicHermiteSpline({-22, -18}, {25, -82}, {3, -62}, {1, -128.5});
    CubicHermiteSpline ring3Q3Spline = CubicHermiteSpline({3.5, -47.5}, {-87, -46}, {-60, -47}, {-86.5, -46.5});
    CubicHermiteSpline ringFarQ3Spline = CubicHermiteSpline({-56, -46.5}, {-64.5, -96}, {-37, -60}, {-8, -59});
    CubicHermiteSpline cornerQ3Spline = CubicHermiteSpline({-37, -60}, {-80.5, -60}, {-63.5, -63.5}, {-80.3, -80});

    CubicHermiteSpline crossQ34Spline = CubicHermiteSpline({-50, -62}, {-52.5, 62}, {-23.5, 23.5}, {-4, 51.5});

    CubicHermiteSpline goalQ4Spline = CubicHermiteSpline({-23.5, 23.5}, {-62, 22.5}, {-49.5, 23.5}, {-62, 22.5});
    CubicHermiteSpline wallQ4Spline = CubicHermiteSpline({-49.5, 23.5}, {-17, 18}, {5.5, 60}, {1, 142});
    CubicHermiteSpline ring3Q4Spline = CubicHermiteSpline({5.5, 47.5}, {-78, 47}, {-56.5, 47}, {-77.5, 47.5});
    CubicHermiteSpline ringFarQ4Spline = CubicHermiteSpline({-56.5, 47}, {-60, 90}, {-47, 60}, {37, 60.5});
    CubicHermiteSpline cornerQ4Spline = CubicHermiteSpline({-47, 60}, {-71, 61.5}, {-63.5, 62}, {-72.5, 73});

    CubicHermiteSpline crossQ41Spline = CubicHermiteSpline({-63, 62}, {18.5, 9.5}, {23, 47}, {53, 46.5});

    CubicHermiteSpline sweepGoalQ12Spline = CubicHermiteSpline({23, 47}, {69, 53}, {63, -64}, {58, -138});
    CubicHermiteSpline goalQ12Spline = CubicHermiteSpline({23, 47}, {69, 53}, {63, -64.5}, {58, -138});
    // CubicHermiteSpline cornerQ1Spline = CubicHermiteSpline();

    // profile setup
    MotionProfile* goalQ3Profile = new MotionProfile(&goalQ3Spline, RPMtoIPS(600));
    MotionProfile* ringMidQ3Profile = new MotionProfile(&ringMidQ3Spline, RPMtoIPS(600));
    MotionProfile* wallQ3Profile = new MotionProfile(&wallQ3Spline, RPMtoIPS(600));
    MotionProfile* ring3Q3Profile = new MotionProfile(&ring3Q3Spline, RPMtoIPS(600), 
        {
            {{0, 0.1}, {0.1, 0.8}},
            {{0.1, 0.8}, {0.9, 0.8}},
            {{0.9, 0.8}, {1, 0}}
        }
    );
    MotionProfile* ringFarQ3Profile = new MotionProfile(&ringFarQ3Spline, RPMtoIPS(600));
    MotionProfile* cornerQ3Profile = new MotionProfile(&cornerQ3Spline, RPMtoIPS(600));

    MotionProfile* crossQ34Profile = new MotionProfile(&crossQ34Spline, RPMtoIPS(600));

    MotionProfile* goalQ4Profile = new MotionProfile(&goalQ4Spline, RPMtoIPS(600));
    MotionProfile* wallQ4Profile = new MotionProfile(&wallQ4Spline, RPMtoIPS(600));
    MotionProfile* ring3Q4Profile = new MotionProfile(&ring3Q4Spline, RPMtoIPS(600));
    MotionProfile* ringFarQ4Profile = new MotionProfile(&ringFarQ4Spline, RPMtoIPS(600));
    MotionProfile* cornerQ4Profile = new MotionProfile(&cornerQ4Spline, RPMtoIPS(600));

    MotionProfile* crossQ41Profile = new MotionProfile(&crossQ41Spline, RPMtoIPS(600));

    MotionProfile* sweepGoalQ12Profile = new MotionProfile(&sweepGoalQ12Spline, RPMtoIPS(600));
    MotionProfile* goalQ12Profile = new MotionProfile(&goalQ12Spline, RPMtoIPS(600));
    // MotionProfile* cornerQ1Profile = new MotionProfile(&cornerQ1Spline, RPMtoIPS(600));

    return {goalQ3Profile, ringMidQ3Profile, wallQ3Profile, ring3Q3Profile, ringFarQ3Profile, 
            cornerQ3Profile, crossQ34Profile, goalQ4Profile, wallQ4Profile, ring3Q4Profile, 
            ringFarQ4Profile, cornerQ4Profile, crossQ41Profile, sweepGoalQ12Profile, goalQ12Profile, 
            goalQ12Profile};
}