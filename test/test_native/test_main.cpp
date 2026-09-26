#include <unity.h>
#include "DutyMapping.h"
#include "CurrentSense.h"
#include "ModeSelection.h"
#include "DeadReckoning.h"
#include "PositionDeadband.h"
#include "AnalogTargetMapping.h"

void setUp() {}
void tearDown() {}

// ---- DutyMapping (specs/exhaust-valve/pwm-target-input) ----

// cfg::INVERT_TARGET_DUTY = true on this ECU: 0% duty maps to fully open,
// 100% to fully closed.

void test_duty_zero_is_fully_open()
{
    TEST_ASSERT_EQUAL_FLOAT(1.0f, dutyPercentToTargetFraction(0.0f));
}

void test_duty_fifty_is_half_open()
{
    TEST_ASSERT_EQUAL_FLOAT(0.5f, dutyPercentToTargetFraction(50.0f));
}

void test_duty_hundred_is_fully_closed()
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, dutyPercentToTargetFraction(100.0f));
}

void test_duty_out_of_range_clamps()
{
    TEST_ASSERT_EQUAL_FLOAT(1.0f, dutyPercentToTargetFraction(-15.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, dutyPercentToTargetFraction(140.0f));
}

// ---- CurrentSense (design.md - current-sense scaling: 0.00V=0A, 4.0V=3.4A) ----

void test_current_zero_counts_is_zero_amps()
{
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, adcCountsToAmps(0));
}

void test_current_max_scale_counts_is_max_amps()
{
    // 4.0V at 5V AREF / 1023 counts ~= 818 counts
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 3.4f, adcCountsToAmps(818));
}

void test_current_half_scale_counts_is_half_max_amps()
{
    // 2.0V at 5V AREF / 1023 counts ~= 409 counts
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.7f, adcCountsToAmps(409));
}

// ---- ModeSelection (specs/exhaust-valve/homing-calibration) ----

void test_mode_selection_meaningful_range_selects_position()
{
    TEST_ASSERT_TRUE(selectControlMode(93, 536, 100) == ControlMode::Position);
}

void test_mode_selection_flat_range_selects_time()
{
    TEST_ASSERT_TRUE(selectControlMode(500, 550, 100) == ControlMode::Time);
}

void test_mode_selection_is_order_independent()
{
    // closed/open readings might not be monotonic depending on wiring polarity.
    TEST_ASSERT_TRUE(selectControlMode(536, 93, 100) == ControlMode::Position);
}

// ---- DeadReckoning (specs/exhaust-valve/valve-position-control - Open-loop time-based control) ----

void test_dead_reckoning_forward_accumulation()
{
    DeadReckoningEstimator estimator(1000, 2000, 0.0f);
    estimator.update(1, 500);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, estimator.fraction());
}

void test_dead_reckoning_direction_reversal_mid_travel()
{
    DeadReckoningEstimator estimator(1000, 2000, 0.0f);
    estimator.update(1, 1000); // fully open
    estimator.update(-1, 1000); // half of the 2000ms close time
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, estimator.fraction());
}

void test_dead_reckoning_clamps_at_bounds()
{
    DeadReckoningEstimator estimator(1000, 2000, 0.0f);
    estimator.update(1, 5000); // way past fully open
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, estimator.fraction());

    estimator.update(-1, 10000); // way past fully closed
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, estimator.fraction());
}

// ---- PositionDeadband (specs/exhaust-valve/valve-position-control - Position deadband) ----

void test_deadband_no_drive_when_within_band()
{
    TEST_ASSERT_FALSE(shouldDrive(0.50f, 0.50f, 0.02f));
    TEST_ASSERT_FALSE(shouldDrive(0.50f, 0.52f, 0.02f)); // exactly at the boundary
}

void test_deadband_drives_when_beyond_band()
{
    TEST_ASSERT_TRUE(shouldDrive(0.50f, 0.53f, 0.02f));
}

// ---- AnalogTargetMapping (bench potentiometer target simulation, A4) ----

void test_analog_target_zero_counts_is_zero_percent()
{
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, analogCountsToPercent(0, 900));
}

void test_analog_target_max_counts_is_hundred_percent()
{
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, analogCountsToPercent(900, 900));
}

void test_analog_target_half_counts_is_fifty_percent()
{
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, analogCountsToPercent(450, 900));
}

void test_analog_target_clamps_above_max_counts()
{
    // The ADC can read up to 1023, past the 900-count "100%" mark.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, analogCountsToPercent(1023, 900));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_duty_zero_is_fully_open);
    RUN_TEST(test_duty_fifty_is_half_open);
    RUN_TEST(test_duty_hundred_is_fully_closed);
    RUN_TEST(test_duty_out_of_range_clamps);

    RUN_TEST(test_current_zero_counts_is_zero_amps);
    RUN_TEST(test_current_max_scale_counts_is_max_amps);
    RUN_TEST(test_current_half_scale_counts_is_half_max_amps);

    RUN_TEST(test_mode_selection_meaningful_range_selects_position);
    RUN_TEST(test_mode_selection_flat_range_selects_time);
    RUN_TEST(test_mode_selection_is_order_independent);

    RUN_TEST(test_dead_reckoning_forward_accumulation);
    RUN_TEST(test_dead_reckoning_direction_reversal_mid_travel);
    RUN_TEST(test_dead_reckoning_clamps_at_bounds);

    RUN_TEST(test_deadband_no_drive_when_within_band);
    RUN_TEST(test_deadband_drives_when_beyond_band);

    RUN_TEST(test_analog_target_zero_counts_is_zero_percent);
    RUN_TEST(test_analog_target_max_counts_is_hundred_percent);
    RUN_TEST(test_analog_target_half_counts_is_fifty_percent);
    RUN_TEST(test_analog_target_clamps_above_max_counts);

    return UNITY_END();
}
