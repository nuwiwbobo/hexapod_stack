// =============================================================================
// test_per_leg_ik.cpp — Per-leg IK tests for all 6 hexapod legs
// =============================================================================
// Tests IK for each leg (LF, RF, LM, RM, LR, RR) with:
//   - Leg-frame foot positions (IK solver operates in leg frame)
//   - AX-18A joint angle limits (±2.618 rad / 300°)
//   - FK round-trip verification
//   - Sign-convention → servo tick conversion
//   - Left-right coxa mirror symmetry
//   - Body-frame → leg-frame mounting offset conversion (future integration)
//
// Run: colcon build --packages-select hexapod_ik && colcon test --packages-select hexapod_ik

#include <gtest/gtest.h>
#include <hexapod_ik/ik_solver.hpp>
#include <cmath>
#include <algorithm>

using namespace hexapod_ik;

// ---------------------------------------------------------------------------
// Per-leg configuration
// ---------------------------------------------------------------------------
struct LegMount {
  std::string name;
  double mx, my;     // body-frame coxa position (m)
  int sx, sf, st;    // sign: coxa, femur, tibia
};

// Approximate body-frame coxa positions for a ~30 cm hexapod.
// Sign conventions per AGENTS.md.
static const LegMount ALL_LEGS[] = {
  {"LF",  0.07,  0.12, +1, +1, -1},
  {"RF", -0.07,  0.12, -1, -1, +1},
  {"LM",  0.09,  0.00, +1, +1, -1},
  {"RM", -0.09,  0.00, -1, -1, +1},
  {"LR",  0.07, -0.12, +1, +1, -1},
  {"RR", -0.07, -0.12, -1, -1, +1},
};

// Body-frame foot → leg-frame foot
static FootPosition bodyToLeg(const FootPosition& bf, const LegMount& leg) {
  return {bf.x - leg.mx, bf.y - leg.my, bf.z};
}

// AX-18A: 1024 ticks, center 512, 5.236 rad, offset 0.00614 rad
static uint16_t angleToTick(double rad, int sign) {
  constexpr int    TICKS   = 1024;
  constexpr int    CENTER  = 512;
  constexpr double MAX_RAD = 5.236;
  constexpr double OFFSET  = 0.00614;
  double raw = CENTER + sign * (rad - OFFSET) * (TICKS / MAX_RAD);
  int c = static_cast<int>(std::round(raw));
  c = std::max(0, std::min(c, TICKS));
  return static_cast<uint16_t>(c);
}

// ---------------------------------------------------------------------------
// Parameterized per-leg tests — use leg-frame foot positions directly
// ---------------------------------------------------------------------------
class PerLegIKTest : public ::testing::TestWithParam<LegMount> {
protected:
  void SetUp() override {
    solver_ = std::make_unique<IkSolver>(LegParams{0.044, 0.0545, 0.1019});
  }
  std::unique_ptr<IkSolver> solver_;
};

INSTANTIATE_TEST_SUITE_P(AllSixLegs, PerLegIKTest,
  ::testing::Values(ALL_LEGS[0], ALL_LEGS[1], ALL_LEGS[2],
                    ALL_LEGS[3], ALL_LEGS[4], ALL_LEGS[5]));

// Default stance position reachable for all legs
TEST_P(PerLegIKTest, StanceReachable) {
  auto r = solver_->solve({0.0, 0.10, -0.10});
  EXPECT_EQ(r.status, IKStatus::SUCCESS) << GetParam().name;
}

// Joint angles within AX-18A physical limits; FK round-trip
TEST_P(PerLegIKTest, JointAnglesInLimits) {
  auto r = solver_->solve({0.0, 0.10, -0.10});
  ASSERT_EQ(r.status, IKStatus::SUCCESS) << GetParam().name;

  // AX-18A: 300° / ±2.618 rad range
  EXPECT_GE(r.joints.coxa,  -2.618);
  EXPECT_LE(r.joints.coxa,   2.618);
  EXPECT_GE(r.joints.femur, -2.618);
  EXPECT_LE(r.joints.femur,  0.0);
  EXPECT_GE(r.joints.tibia,  0.0);
  EXPECT_LE(r.joints.tibia,  2.618);

  auto rt = solver_->forward(r.joints);
  EXPECT_NEAR(rt.x, 0.0, 0.001);
  EXPECT_NEAR(rt.y, 0.10, 0.001);
  EXPECT_NEAR(rt.z, -0.10, 0.001);
}

// Sign convention maps IK angles to valid servo ticks
TEST_P(PerLegIKTest, SignConversion) {
  auto r = solver_->solve({0.0, 0.10, -0.10});
  ASSERT_EQ(r.status, IKStatus::SUCCESS) << GetParam().name;

  auto& leg = GetParam();
  uint16_t ct = angleToTick(r.joints.coxa,  leg.sx);
  uint16_t ft = angleToTick(r.joints.femur, leg.sf);
  uint16_t tt = angleToTick(r.joints.tibia, leg.st);

  EXPECT_GE(ct, 0);   EXPECT_LE(ct, 1024);
  EXPECT_GE(ft, 0);   EXPECT_LE(ft, 1024);
  EXPECT_GE(tt, 0);   EXPECT_LE(tt, 1024);
}

// Multiple leg-frame stance positions
TEST_P(PerLegIKTest, MultipleStancePositions) {
  std::vector<FootPosition> feet = {
    {0.0,   0.10, -0.10},
    {0.04,  0.12, -0.10},
    {-0.04, 0.08, -0.10},
    {0.0,   0.14, -0.08},
    {0.0,   0.06, -0.10},
  };
  for (auto& f : feet) {
    auto r = solver_->solve(f);
    EXPECT_EQ(r.status, IKStatus::SUCCESS)
      << GetParam().name << " foot (" << f.x << ", " << f.y << ", " << f.z << ")";
  }
}

// Positions well beyond reach return UNREACHABLE
TEST_P(PerLegIKTest, Unreachable) {
  // max_reach = 0.0545 + 0.1019 = 0.1564 m
  // far = 0.25 m → clearly beyond reach
  for (auto& f : {FootPosition{0.0, 0.0, -0.25},
                  FootPosition{0.0, 0.25,  0.0},
                  FootPosition{0.25, 0.0,  0.0}}) {
    auto r = solver_->solve(f);
    EXPECT_EQ(r.status, IKStatus::UNREACHABLE)
      << GetParam().name << " should be UNREACHABLE at ("
      << f.x << ", " << f.y << ", " << f.z << ")";
  }
}

// ---------------------------------------------------------------------------
// Symmetry: left-right leg pairs with mirror foot positions
// ---------------------------------------------------------------------------
class SymmetryTest : public ::testing::Test {
protected:
  void SetUp() override {
    solver_ = std::make_unique<IkSolver>(LegParams{0.044, 0.0545, 0.1019});
  }
  std::unique_ptr<IkSolver> solver_;
};

TEST_F(SymmetryTest, CoxaMirrorForOppositeX) {
  // Same Y, opposite X foot → coxa should be mirror-symmetric
  for (double fx : {0.03, 0.05, 0.08}) {
    FootPosition posL{fx, 0.10, -0.10};
    FootPosition posR{-fx, 0.10, -0.10};

    auto rL = solver_->solve(posL);
    auto rR = solver_->solve(posR);
    ASSERT_EQ(rL.status, IKStatus::SUCCESS);
    ASSERT_EQ(rR.status, IKStatus::SUCCESS);

    EXPECT_NEAR(rL.joints.coxa, -rR.joints.coxa, 0.001)
      << "coxa not mirror at fx=" << fx;
    EXPECT_NEAR(rL.joints.femur, rR.joints.femur, 0.001)
      << "femur not equal at fx=" << fx;
    EXPECT_NEAR(rL.joints.tibia, rR.joints.tibia, 0.001)
      << "tibia not equal at fx=" << fx;
  }
}

TEST_F(SymmetryTest, CoxaSignMatchesFootXSign) {
  // Positive foot X → positive coxa; negative foot X → negative coxa
  auto r_pos = solver_->solve({0.05, 0.10, -0.10});
  auto r_neg = solver_->solve({-0.05, 0.10, -0.10});
  ASSERT_EQ(r_pos.status, IKStatus::SUCCESS);
  ASSERT_EQ(r_neg.status, IKStatus::SUCCESS);

  EXPECT_GT(r_pos.joints.coxa, 0.0);
  EXPECT_LT(r_neg.joints.coxa, 0.0);
}

TEST_F(SymmetryTest, ZeroFootXGivesZeroCoxa) {
  auto r = solver_->solve({0.0, 0.10, -0.10});
  ASSERT_EQ(r.status, IKStatus::SUCCESS);
  EXPECT_NEAR(r.joints.coxa, 0.0, 0.001);
}

// ---------------------------------------------------------------------------
// Mounting offset tests — body-frame foot positions converted per leg
// ---------------------------------------------------------------------------
class MountOffsetTest : public ::testing::Test {
protected:
  void SetUp() override {
    solver_ = std::make_unique<IkSolver>(LegParams{0.044, 0.0545, 0.1019});
  }
  std::unique_ptr<IkSolver> solver_;

  const LegMount& findLeg(const std::string& name) const {
    for (auto& l : ALL_LEGS) if (l.name == name) return l;
    return ALL_LEGS[0]; // fallback (should never happen in tests)
  }
};

// All legs can reach a body-center stance when offsets are applied
TEST_F(MountOffsetTest, BodyCenterStanceAllLegs) {
  // body-center foot position: all feet near body center, Y forward
  FootPosition bf{0.0, 0.0, -0.10};

  for (auto& leg : ALL_LEGS) {
    auto lf = bodyToLeg(bf, leg);
    auto r = solver_->solve(lf);
    EXPECT_EQ(r.status, IKStatus::SUCCESS)
      << leg.name << " body-foot (0, 0, -0.10) → leg-foot ("
      << lf.x << ", " << lf.y << ", " << lf.z << ")";

    if (r.status == IKStatus::SUCCESS) {
      auto rt = solver_->forward(r.joints);
      EXPECT_NEAR(rt.x, lf.x, 0.001) << leg.name;
      EXPECT_NEAR(rt.y, lf.y, 0.001) << leg.name;
      EXPECT_NEAR(rt.z, lf.z, 0.001) << leg.name;
    }
  }
}

// Legs at different Y mounts produce different coxa signs for same body-foot
TEST_F(MountOffsetTest, FrontLegsPositiveCoxa) {
  // Front legs (LF, RF) with mount_y=0.12 and body-foot y=0.0:
  //   leg-foot y = 0.0 - 0.12 = -0.12 (foot behind coxa)
  //   For LF: leg-foot x < 0 → coxa < 0 (points backward)
  FootPosition bf{0.0, 0.0, -0.10};

  auto lfLF = bodyToLeg(bf, findLeg("LF"));
  auto lfRF = bodyToLeg(bf, findLeg("RF"));
  auto rLF = solver_->solve(lfLF);
  auto rRF = solver_->solve(lfRF);
  ASSERT_EQ(rLF.status, IKStatus::SUCCESS);
  ASSERT_EQ(rRF.status, IKStatus::SUCCESS);

  EXPECT_LT(rLF.joints.coxa, 0.0) << "LF coxa should be negative (foot behind)";
  EXPECT_GT(rRF.joints.coxa, 0.0) << "RF coxa should be positive (foot behind)";
}

// Mounting offset symmetry: symmetric body-foot positions produce mirror coxa
TEST_F(MountOffsetTest, CoxaMirrorThroughMountOffset) {
  // LF: mount (0.07, 0.12), RF: mount (-0.07, 0.12)
  // body-foot (0, 0, -0.10):
  //   LF leg-foot: (-0.07, -0.12, -0.10)
  //   RF leg-foot: (0.07, -0.12, -0.10)
  FootPosition bf{0.0, 0.0, -0.10};
  auto lfL = bodyToLeg(bf, findLeg("LF"));
  auto lfR = bodyToLeg(bf, findLeg("RF"));
  auto rL = solver_->solve(lfL);
  auto rR = solver_->solve(lfR);
  ASSERT_EQ(rL.status, IKStatus::SUCCESS);
  ASSERT_EQ(rR.status, IKStatus::SUCCESS);

  EXPECT_NEAR(rL.joints.coxa, -rR.joints.coxa, 0.001)
    << "LF/RF coxa not mirror through mount offset";
  EXPECT_NEAR(rL.joints.femur, rR.joints.femur, 0.001);
  EXPECT_NEAR(rL.joints.tibia, rR.joints.tibia, 0.001);
}
