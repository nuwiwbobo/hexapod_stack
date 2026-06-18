// =============================================================================
// Task 5: Unit tests for IK Solver
// =============================================================================
// TDD approach: write these tests first, then implement ik_solver.cpp
//
// Test list:
//   1. FkRoundTrip - known joints → FK → IK → compare (error < 0.001 rad)
//   2. ZeroPosition - foot at default stance → joint angles near zero
//   3. UnreachableTarget - foot at (1.0, 0, 0) → returns UNREACHABLE
//   4. BoundaryReach - foot at exact max reach → returns valid angles
//   5. MultipleAngles - test several different joint configurations
//
// Run: colcon build --packages-select hexapod_ik && colcon test --packages-select hexapod_ik

#include <gtest/gtest.h>
#include <hexapod_ik/ik_solver.hpp>
#include <cmath>

using namespace hexapod_ik;

class IkSolverTest : public ::testing::Test {
protected:
  void SetUp() override {
    LegParams params;
    params.coxa_length = 0.044;
    params.femur_length = 0.0545;
    params.tibia_length = 0.1019;
    solver_ = std::make_unique<IkSolver>(params);
  }

  std::unique_ptr<IkSolver> solver_;
};

// TODO: Implement test - FK round-trip should recover original joint angles
TEST_F(IkSolverTest, FkRoundTrip) {
  LegJoints original;
  original.coxa = 0.3;
  original.femur = -0.5;
  original.tibia = 0.8;

  FootPosition foot = solver_->forward(original);
  IKResult result = solver_->solve(foot);

  EXPECT_EQ(result.status, IKStatus::SUCCESS);
  EXPECT_NEAR(result.joints.coxa, original.coxa, 0.001);
  EXPECT_NEAR(result.joints.femur, original.femur, 0.001);
  EXPECT_NEAR(result.joints.tibia, original.tibia, 0.001);
}

// TODO: Implement test - zero position should round-trip correctly
TEST_F(IkSolverTest, ZeroPosition) {
  LegJoints zero;
  zero.coxa = 0.0;
  zero.femur = 0.0;
  zero.tibia = 0.0;

  FootPosition foot = solver_->forward(zero);
  IKResult result = solver_->solve(foot);

  EXPECT_EQ(result.status, IKStatus::SUCCESS);
  EXPECT_NEAR(result.joints.coxa, 0.0, 0.001);
  EXPECT_NEAR(result.joints.femur, 0.0, 0.001);
  EXPECT_NEAR(result.joints.tibia, 0.0, 0.001);
}

// TODO: Implement test - unreachable target should return UNREACHABLE status
TEST_F(IkSolverTest, UnreachableTarget) {
  FootPosition foot;
  foot.x = 1.0;
  foot.y = 0.0;
  foot.z = 0.0;

  IKResult result = solver_->solve(foot);
  EXPECT_EQ(result.status, IKStatus::UNREACHABLE);
}

// TODO: Implement test - exact max reach should return valid angles
TEST_F(IkSolverTest, BoundaryReach) {
  LegJoints max_reach;
  max_reach.coxa = 0.0;
  max_reach.femur = 0.0;
  max_reach.tibia = 0.0;

  FootPosition foot = solver_->forward(max_reach);
  IKResult result = solver_->solve(foot);
  EXPECT_EQ(result.status, IKStatus::SUCCESS);
}

// TODO: Implement test - multiple joint configurations should all round-trip
TEST_F(IkSolverTest, MultipleAngles) {
  std::vector<LegJoints> configs = {
    {0.0, -0.3, 0.5},
    {0.5, -0.8, 1.0},
    {-0.5, -0.2, 0.3},
    {0.0, -1.0, 1.5},
  };

  for (const auto& original : configs) {
    FootPosition foot = solver_->forward(original);
    IKResult result = solver_->solve(foot);

    EXPECT_EQ(result.status, IKStatus::SUCCESS)
      << "Failed for coxa=" << original.coxa
      << " femur=" << original.femur
      << " tibia=" << original.tibia;
    EXPECT_NEAR(result.joints.coxa, original.coxa, 0.001);
    EXPECT_NEAR(result.joints.femur, original.femur, 0.001);
    EXPECT_NEAR(result.joints.tibia, original.tibia, 0.001);
  }
}
