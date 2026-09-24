// Faz 2 sözleşmesi — include/limo_base/limo_kinematics.hpp
//
// Sabit sayılar Python ile bağımsız hesaplandı (sürücü sabitleri
// limo_driver.h:130-132: L = 0.2, T = 0.172, max_inner = 0.48869).

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "limo_base/limo_kinematics.hpp"

using namespace AgileX::kinematics;

namespace
{
constexpr double L = 0.2;
constexpr double T = 0.172;
constexpr double MAX_INNER = 0.48869;
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double INF = std::numeric_limits<double>::infinity();
constexpr double EPS = 1e-9;
}  // namespace

// ------------------------------------------------------------ açı dönüşümleri

TEST(SteerConversion, InnerToCentralKnownValue)
{
  EXPECT_NEAR(innerToCentral(MAX_INNER, L, T), 0.40842711653157876, EPS);
  EXPECT_NEAR(innerToCentral(-MAX_INNER, L, T), -0.40842711653157876, EPS);
}

TEST(SteerConversion, ZeroIsZero)
{
  EXPECT_EQ(innerToCentral(0.0, L, T), 0.0);
  EXPECT_EQ(centralToInner(0.0, L, T), 0.0);
}

TEST(SteerConversion, RoundTrip)
{
  for (double inner : {0.02, 0.1, 0.25, 0.4, MAX_INNER, -0.3, -0.05}) {
    EXPECT_NEAR(centralToInner(innerToCentral(inner, L, T), L, T), inner, EPS) << inner;
  }
}

// ------------------------------------------------------------ cmd_vel -> direksiyon (D1)

TEST(TwistToInnerSteering, StraightWhenNoYawRate)
{
  double inner = 123.0;
  ASSERT_TRUE(twistToInnerSteering(0.5, 0.0, L, T, MAX_INNER, &inner));
  EXPECT_EQ(inner, 0.0);
}

TEST(TwistToInnerSteering, StopCommandIsFiniteZero)
{
  // Upstream: 0.0 / 0.0 = NaN -> int16 dönüşümü (tanımsız davranış).
  // Nav2 hedefe varınca tam olarak bunu yayınlar.
  double inner = 123.0;
  ASSERT_TRUE(twistToInnerSteering(0.0, 0.0, L, T, MAX_INNER, &inner));
  EXPECT_EQ(inner, 0.0);
}

TEST(TwistToInnerSteering, PureRotationRequestLocksWithSignOfW)
{
  double inner = 0.0;
  ASSERT_TRUE(twistToInnerSteering(0.0, 0.7, L, T, MAX_INNER, &inner));
  EXPECT_NEAR(inner, MAX_INNER, EPS);
  ASSERT_TRUE(twistToInnerSteering(0.0, -0.7, L, T, MAX_INNER, &inner));
  EXPECT_NEAR(inner, -MAX_INNER, EPS);
}

TEST(TwistToInnerSteering, KnownCurvature)
{
  double inner = 0.0;
  // r = v / w = 1.0 m
  ASSERT_TRUE(twistToInnerSteering(0.5, 0.5, L, T, MAX_INNER, &inner));
  EXPECT_NEAR(inner, 0.21542295681067072, EPS);
}

TEST(TwistToInnerSteering, ReverseFlipsSign)
{
  double inner = 0.0;
  ASSERT_TRUE(twistToInnerSteering(-0.5, 0.5, L, T, MAX_INNER, &inner));
  EXPECT_NEAR(inner, -0.21542295681067072, EPS);
}

TEST(TwistToInnerSteering, ClampsToMaxInner)
{
  double inner = 0.0;
  // r = 0.3 -> sınırsız iç açı 0.7516 rad, 0.48869'a kırpılmalı
  ASSERT_TRUE(twistToInnerSteering(0.3, 1.0, L, T, MAX_INNER, &inner));
  EXPECT_NEAR(inner, MAX_INNER, EPS);
  ASSERT_TRUE(twistToInnerSteering(0.3, -1.0, L, T, MAX_INNER, &inner));
  EXPECT_NEAR(inner, -MAX_INNER, EPS);
}

TEST(TwistToInnerSteering, TinyRadiusIsClampedNotNaN)
{
  double inner = 0.0;
  // |r| < T/2 dalı
  ASSERT_TRUE(twistToInnerSteering(0.01, 2.0, L, T, MAX_INNER, &inner));
  EXPECT_TRUE(std::isfinite(inner));
  EXPECT_NEAR(inner, MAX_INNER, EPS);
}

TEST(TwistToInnerSteering, RejectsNonFiniteAndLeavesOutputUntouched)
{
  for (auto vw : {std::pair<double, double>{NaN, 0.1}, {0.1, NaN}, {INF, 0.1}, {0.1, -INF}}) {
    double inner = 42.0;
    EXPECT_FALSE(twistToInnerSteering(vw.first, vw.second, L, T, MAX_INNER, &inner));
    EXPECT_EQ(inner, 42.0);
  }
}

TEST(TwistToInnerSteering, NeverProducesNaNOverGrid)
{
  for (double v = -1.0; v <= 1.0; v += 0.05) {
    for (double w = -3.0; w <= 3.0; w += 0.05) {
      double inner = 0.0;
      ASSERT_TRUE(twistToInnerSteering(v, w, L, T, MAX_INNER, &inner));
      ASSERT_TRUE(std::isfinite(inner)) << "v=" << v << " w=" << w;
      ASSERT_LE(std::fabs(inner), MAX_INNER + EPS) << "v=" << v << " w=" << w;
    }
  }
}

// ------------------------------------------------------------ odometri yaw hızı

TEST(AckermannYawRate, KnownValuesAndSigns)
{
  EXPECT_NEAR(ackermannYawRate(0.5, 0.2, L, T), 0.46614354013530573, EPS);
  EXPECT_NEAR(ackermannYawRate(-0.5, 0.2, L, T), -0.46614354013530573, EPS);
  EXPECT_NEAR(ackermannYawRate(0.5, -0.2, L, T), -0.46614354013530573, EPS);
}

TEST(AckermannYawRate, StraightAndNonFinite)
{
  EXPECT_EQ(ackermannYawRate(0.5, 0.0, L, T), 0.0);
  EXPECT_EQ(ackermannYawRate(NaN, 0.2, L, T), 0.0);
  EXPECT_EQ(ackermannYawRate(0.5, NaN, L, T), 0.0);
}

// ------------------------------------------------------------ protokol komutu

TEST(ToCommand, ScalesAndRounds)
{
  EXPECT_EQ(toCommand(0.5), 500);
  EXPECT_EQ(toCommand(-0.25), -250);
  EXPECT_EQ(toCommand(0.0004), 0);
  EXPECT_EQ(toCommand(0.0006), 1);
  EXPECT_EQ(toCommand(-0.0006), -1);
}

TEST(ToCommand, SaturatesAndRejectsNonFinite)
{
  EXPECT_EQ(toCommand(40.0), 32767);
  EXPECT_EQ(toCommand(-40.0), -32768);
  EXPECT_EQ(toCommand(NaN), 0);
  EXPECT_EQ(toCommand(INF), 0);
  EXPECT_EQ(toCommand(-INF), 0);
}

// ------------------------------------------------------------ bayt çözme

TEST(DecodeInt32BE, PositiveAndNegative)
{
  const uint8_t pos[4] = {0x00, 0x01, 0x02, 0x03};
  EXPECT_EQ(decodeInt32BE(pos), 0x00010203);
  const uint8_t minus_one[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  EXPECT_EQ(decodeInt32BE(minus_one), -1);
  const uint8_t min_val[4] = {0x80, 0x00, 0x00, 0x00};
  EXPECT_EQ(decodeInt32BE(min_val), std::numeric_limits<int32_t>::min());
  const uint8_t neg[4] = {0xFF, 0xFF, 0xFE, 0x0C};  // -500
  EXPECT_EQ(decodeInt32BE(neg), -500);
}

// ------------------------------------------------------------ IMU yaw adımı (D2)

TEST(YawDeltaDeg, PlainDifference)
{
  EXPECT_NEAR(yawDeltaDeg(10.0, 7.5, 0.0), 2.5, EPS);
  EXPECT_NEAR(yawDeltaDeg(7.5, 10.0, 0.0), -2.5, EPS);
}

TEST(YawDeltaDeg, WrapsAcross180)
{
  EXPECT_NEAR(yawDeltaDeg(-179.0, 179.0, 0.0), 2.0, EPS);
  EXPECT_NEAR(yawDeltaDeg(179.0, -179.0, 0.0), -2.0, EPS);
}

TEST(YawDeltaDeg, DeadBand)
{
  // Upstream sabit 0.1° ölü bant: 100 Hz'de ~10°/s altındaki dönüşleri siliyordu.
  EXPECT_EQ(yawDeltaDeg(10.05, 10.0, 0.1), 0.0);
  EXPECT_NEAR(yawDeltaDeg(10.05, 10.0, 0.0), 0.05, EPS);
  EXPECT_NEAR(yawDeltaDeg(10.05, 10.0, -1.0), 0.05, EPS);  // negatif = ölü bant yok
  EXPECT_NEAR(yawDeltaDeg(10.2, 10.0, 0.1), 0.2, EPS);
}

TEST(YawDeltaDeg, NonFiniteIsZero)
{
  EXPECT_EQ(yawDeltaDeg(NaN, 1.0, 0.0), 0.0);
  EXPECT_EQ(yawDeltaDeg(1.0, INF, 0.0), 0.0);
}

// ------------------------------------------------------------ uçtan uca: dur komutu

TEST(EndToEnd, StopTwistYieldsZeroSteeringCommand)
{
  double inner = 1.0;
  ASSERT_TRUE(twistToInnerSteering(0.0, 0.0, L, T, MAX_INNER, &inner));
  const double angle_scale = 2.47;  // limo_driver.h:134
  EXPECT_EQ(toCommand(inner / angle_scale), 0);
  EXPECT_EQ(toCommand(0.0), 0);
}
