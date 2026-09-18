#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "forward_comp.h"
#include "motion.h"
#include "zdt_x42s.h"

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

static uint32_t float_bits(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static ForwardCompResult apply(float vx, float vy, float lateral_direction, float k)
{
    return ForwardComp_Apply(vx, vy, lateral_direction, 1.25f, 0.75f, k);
}

static void test_zero_gain_preserves_numeric_and_wire_output(void)
{
    ForwardCompResult result = apply(100.0f, 0.0f, 0.0f, 0.0f);
    assert_close(result.vy_original, 0.0f);
    assert_close(result.forward_comp_vy, 0.0f);
    assert_close(result.vy_final, 0.0f);

    float rpm[4];
    Mecanum_Inverse((Geometry){35.0f, 259.0f}, 100.0f, result.vy_final, 0.0f, rpm);
    for (int i = 0; i < 4; ++i)
        assert_close(rpm[i], 27.2837048f);

    const uint8_t ids[4] = {2, 1, 3, 4};
    const int8_t signs[4] = {1, -1, 1, -1};
    int16_t physical_rpm[4];
    for (int i = 0; i < 4; ++i)
        physical_rpm[i] = (int16_t)((int)roundf(rpm[i]) * signs[i]);
    uint8_t frame[ZDT_MULTI_SPEED_SIZE];
    const uint8_t expected[ZDT_MULTI_SPEED_SIZE] = {
        0, 0xAA, 0, 0x25,
        2, 0xF6, 0, 0x01, 0x0E, 0, 0, 0x6B,
        1, 0xF6, 1, 0x01, 0x0E, 0, 0, 0x6B,
        3, 0xF6, 0, 0x01, 0x0E, 0, 0, 0x6B,
        4, 0xF6, 1, 0x01, 0x0E, 0, 0, 0x6B,
        0x6B};
    assert(ZDT_BuildMultiSpeed(frame, sizeof(frame), ids, physical_rpm, 0) == sizeof(frame));
    assert(memcmp(frame, expected, sizeof(frame)) == 0);
}

static void test_forward_positive_gain_adds_leftward_velocity(void)
{
    ForwardCompResult result = apply(120.0f, 0.0f, 0.0f, 0.025f);
    assert_close(result.forward_comp_vy, 3.0f);
    assert_close(result.vy_final, 3.0f);
}

static void test_non_forward_commands_are_unchanged(void)
{
    ForwardCompResult backward = apply(-120.0f, 0.0f, 0.0f, 0.025f);
    ForwardCompResult rotate = apply(0.0f, 0.0f, 0.0f, 0.025f);
    assert_close(backward.forward_comp_vy, 0.0f);
    assert_close(backward.vy_final, 0.0f);
    assert_close(rotate.forward_comp_vy, 0.0f);
    assert_close(rotate.vy_final, 0.0f);
}

static void test_existing_lateral_gains_and_order_are_preserved(void)
{
    ForwardCompResult left = apply(0.0f, 20.0f, 1.0f, 0.5f);
    ForwardCompResult right = apply(0.0f, -20.0f, -1.0f, 0.5f);
    assert_close(left.vy_original, 25.0f);
    assert_close(left.vy_final, 25.0f);
    assert_close(right.vy_original, -15.0f);
    assert_close(right.vy_final, -15.0f);
}

static void test_diagonal_direction_never_enables_compensation(void)
{
    ForwardCompResult diagonal = apply(100.0f, 0.25f, 0.0025f, 0.5f);
    assert_close(diagonal.vy_original, 0.3125f);
    assert_close(diagonal.forward_comp_vy, 0.0f);
    assert_close(diagonal.vy_final, 0.3125f);
}

static void test_signed_zero_boundary(void)
{
    ForwardCompResult positive = apply(40.0f, -0.0f, -0.0f, 0.1f);
    ForwardCompResult negative_zero = apply(-0.0f, 0.0f, 0.0f, 0.1f);
    ForwardCompResult zero_gain = apply(40.0f, -0.0f, -0.0f, 0.0f);
    assert_close(positive.forward_comp_vy, 4.0f);
    assert_close(positive.vy_final, 4.0f);
    assert_close(negative_zero.forward_comp_vy, 0.0f);
    assert(float_bits(zero_gain.vy_final) == float_bits(zero_gain.vy_original));
}

static void test_nonfinite_gain_is_invalid(void)
{
    assert(ForwardComp_ConfigValid(0.0f));
    assert(ForwardComp_ConfigValid(-0.1f));
    assert(!ForwardComp_ConfigValid(NAN));
    assert(!ForwardComp_ConfigValid(INFINITY));
    assert(!ForwardComp_ConfigValid(-INFINITY));
}
int main(void)
{
    test_zero_gain_preserves_numeric_and_wire_output();
    test_forward_positive_gain_adds_leftward_velocity();
    test_non_forward_commands_are_unchanged();
    test_existing_lateral_gains_and_order_are_preserved();
    test_diagonal_direction_never_enables_compensation();
    test_signed_zero_boundary();
    test_nonfinite_gain_is_invalid();
    return 0;
}
