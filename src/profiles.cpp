#include "main.h"


std::vector<MotionProfile*> AWPSetup(int color) {
    // spline setup
    // CubicHermiteSpline goalSpline = CubicHermiteSpline({-54.75, 13.25}, {28.7, 45}, {35.14, 29.54}, {11.6, 17.9});
    CubicHermiteSpline goalSpline = CubicHermiteSpline({color * 54.75, 13.25}, {color * -10, 36}, {color * 20, 25}, {color * -10, 36});
    CubicHermiteSpline innerRingSpline = CubicHermiteSpline({color * 20, 25}, {color * 25, 51}, {color * 23, 44}, {color * 29, 83});
    // CubicHermiteSpline outerRingSpline (old) = CubicHermiteSpline({color * 23, 44}, {color * 34, 77}, {color * 50, 39}, {color * 49, 20});
    CubicHermiteSpline outerRingSpline = CubicHermiteSpline({color * 32.75, 58.5}, {color * 82.6, 56}, {color * 53, -12}, {color * 49, -43.5});
    // CubicHermiteSpline crossSpline = CubicHermiteSpline({color * 49.5, 39}, {color * 50, 20}, {color * 50, -12}, {color * 43, -41.5});
    CubicHermiteSpline southernRingSpline = CubicHermiteSpline({color * 53, -11.5}, {color * 56, -120}, {color * 24, -56}, {color * 9.5, -95});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({color * 24, -55.5}, {color * 24.5, -22.5}, {color * 30, -20}, {color * 41, 3});
    CubicHermiteSpline ladder2Spline = CubicHermiteSpline({color * 30, -20}, {color * 41, 3.5}, {color * 13, -12}, {color * 2, -13});

    // profile setup
    MotionProfile* goalProfile = new MotionProfile(&goalSpline, RPMtoIPS(600));
    MotionProfile* innerRingProfile = new MotionProfile(&innerRingSpline, RPMtoIPS(600), 
        {
            {{0, 0.1}, {0.05, 1}}, 
            {{0.05, 1}, {1, 1}}
        }
    );
    /* MotionProfile* outerRingProfile (old) = new MotionProfile(&outerRingSpline, RPMtoIPS(600),
        {
            {{0, 0.1}, {0.1, 1}}, 
            {{0.1, 1}, {1, 1}}
        }
    ); */
    MotionProfile* outerRingProfile = new MotionProfile(&outerRingSpline, RPMtoIPS(600),
        {
            {{0, 1}, {0.9, 1}},
            {{0.5, 1}, {0.7, 0.6}},
            {{0.7, 0.6}, {1, 0.6}}
        }
    );
    MotionProfile* southernRingProfile = new MotionProfile(&southernRingSpline, RPMtoIPS(600), {
        {
            {{0, 0.6}, {0.4, 0.6}},
            {{0.4, 0.6}, {0.5, 1}},
            {{0.5, 1}, {0.9, 1}},
            {{0.9, 1}, {1, 0}}
        }
    });
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, RPMtoIPS(600));
    MotionProfile* ladder2Profile = new MotionProfile(&ladder2Spline, RPMtoIPS(600));

    return {goalProfile, innerRingProfile, outerRingProfile, southernRingProfile, ladderProfile, ladder2Profile};
}

std::vector<MotionProfile*> GoalRushSetup(int color) {
    // spline setup
    CubicHermiteSpline rushSpline = CubicHermiteSpline({color * 50.25, -35}, {color * -8.66, -54.4}, {color * 13.9, -47}, {color * -9.1, -54.8});
    CubicHermiteSpline secondGoalSpline = CubicHermiteSpline({color * 30, -47}, {color * 81.5, -16}, {color * 21.5, -5}, {color * -2, 18});
    CubicHermiteSpline cornerSpline = CubicHermiteSpline({color * 22, -10}, {color * 39, -63}, {color * 54, -63.2}, {color * 81.4, -76});
    CubicHermiteSpline cornerRingSpline = CubicHermiteSpline({color * 51, -63.5}, {color * 127, 24.75}, {color * 55, -10}, {color * 42.25, 14});
    CubicHermiteSpline fetchSpline = CubicHermiteSpline({color * 55, -10}, {color * 1.5, -55.5}, {color * 11, -47.5}, {color * 1.5, -55.5});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({color * 12, -47.5}, {color * 54.5, -10.75}, {color * 13.3, -13.8}, {color * -13.5, 21.7});
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
    MotionProfile* fetchProfile = new MotionProfile(&fetchSpline, RPMtoIPS(600));
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, RPMtoIPS(600));

    return {rushProfile, secondGoalProfile, cornerProfile, cornerRingProfile, fetchProfile, ladderProfile};
}

std::vector<MotionProfile*> RingSetup(int color) {
    // spline setup
    CubicHermiteSpline centerSpline = CubicHermiteSpline({color * 50, 48}, {color * -73.5, -27}, {color * -0.5, 66}, {color * 13.4, 137});
    CubicHermiteSpline goalSpline = CubicHermiteSpline({color * -0.5, 66}, {color * 31, -8}, {color * 27.5, 17.85}, {color * 57.75, -40});
    CubicHermiteSpline cornerSpline = CubicHermiteSpline({color * 24, 22.75}, {color * -18.5, 109.5}, {color * 52.75, 65}, {color * 98.5, 87});
    CubicHermiteSpline sweepSpline = CubicHermiteSpline({color * 52.75, 65}, {color * 122, -4}, {color * 54.25, -2}, {color * 52, -83.25});
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