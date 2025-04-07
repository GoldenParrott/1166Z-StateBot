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
    CubicHermiteSpline newLadderSpline = CubicHermiteSpline({-23, 44}, {-28, 9.3}, {-15, 11}, {1.5, -10.3});

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
    CubicHermiteSpline rushSpline = CubicHermiteSpline({-53, -58.6}, {22.4, -36.5}, {-13.9, -47}, {22.37, -36.3});
    CubicHermiteSpline secondGoalSpline = CubicHermiteSpline({-26.98, -53.06}, {-28.7, -26.3}, {-21.91, -12}, {8.4, 72});
    CubicHermiteSpline cornerSpline = CubicHermiteSpline({-21.9, -12}, {-39, -63}, {-60, -58}, {-69.1, -84.4});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({-60, -58}, {120, -120}, {-21, 3}, {-20, 5});
    // profile setup
    MotionProfile* rushProfile = new MotionProfile(&rushSpline, RPMtoIPS(600),
        {
            {{0, 0.1}, {0.05, 1}}, 
            {{0.05, 1}, {1, 1}}
        }
    );
    MotionProfile* secondGoalProfile = new MotionProfile(&secondGoalSpline, RPMtoIPS(400));
    MotionProfile* cornerProfile = new MotionProfile(&cornerSpline, RPMtoIPS(600));
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, RPMtoIPS(500),
        {
            {{0, 0.1}, {0.1, 1}},
            {{0.1, 1}, {0.4, 0.5}},
            {{0.4, 0.5}, {1, 0.5}}
        }
    );

    return {rushProfile, secondGoalProfile, cornerProfile, ladderProfile};
}

std::vector<MotionProfile*> BlueGoalRushSetup() {
    // spline setup
    CubicHermiteSpline rushSpline = CubicHermiteSpline({50.5, -35.5}, {-17.2, -54.2}, {13.5, -47}, {-17.2, -54.2});
    CubicHermiteSpline secondGoalSpline = CubicHermiteSpline({27.03, -42}, {85.5, -40}, {20, -12}, {9, 0});
    CubicHermiteSpline cornerSpline = CubicHermiteSpline({20, -12}, {81.5, -61}, {60, -65}, {144, -85});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({60, -65}, {128, 105}, {10, -20}, {-2.4, 12.4});
    // profile setup
    MotionProfile* rushProfile = new MotionProfile(&rushSpline, RPMtoIPS(600),
        {
            {{0, 0.1}, {0.05, 1}}, 
            {{0.05, 1}, {1, 1}}
        }
    );
    MotionProfile* secondGoalProfile = new MotionProfile(&secondGoalSpline, RPMtoIPS(400));
    MotionProfile* cornerProfile = new MotionProfile(&cornerSpline, RPMtoIPS(600));
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, RPMtoIPS(500),
        {
            {{0, 0.1}, {0.1, 1}},
            {{0.1, 1}, {0.4, 0.5}},
            {{0.4, 0.5}, {1, 0.5}}
        }
    );

    return {rushProfile, secondGoalProfile, cornerProfile, ladderProfile};
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