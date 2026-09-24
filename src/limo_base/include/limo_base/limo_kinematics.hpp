// LIMO taban sürücüsünün ROS'tan bağımsız, test edilebilir hesapları.
//
// limo_driver.cpp içindeki satır içi hesaplar buraya taşındı ki seri port ve
// ROS düğümü olmadan birim testi yazılabilsin (test/test_limo_kinematics.cpp).
// Bu başlık sözleşmedir: imzalar ve belgelenen davranış değiştirilmemelidir.

#ifndef LIMO_BASE__LIMO_KINEMATICS_HPP_
#define LIMO_BASE__LIMO_KINEMATICS_HPP_

#include <cstdint>

namespace AgileX
{
namespace kinematics
{

// Ackermann: iç tekerlek direksiyon açısını merkez (bisiklet modeli) açısına çevirir.
//   r = L / tan(|inner|) + T/2 ;  central = atan(L / r) ; işaret korunur.
// inner == 0 -> 0.0
double innerToCentral(double inner, double wheelbase, double track);

// innerToCentral'ın tersi:
//   inner = atan(2 L sin|c| / (2 L cos|c| - T sin|c|)) ; işaret korunur.
// central == 0 -> 0.0
double centralToInner(double central, double wheelbase, double track);

// cmd_vel (v [m/s], w [rad/s]) -> iç tekerlek direksiyon açısı [rad].
// Sonuç [-max_inner, +max_inner] aralığına kırpılır.
//
// Davranış (upstream limo_driver.cpp:430-445 ile aynı, NaN hatası hariç):
//   * w == 0                  -> 0.0 (düz git)
//   * v == 0 ve w != 0        -> sign(w) * max_inner (yerinde dönüş istenemez; tam kilit)
//   * aksi halde r = v / w; |r| < T/2 ise r = sign(r) * (T/2 + 0.01) ;
//     central = atan(L / r) ; inner = centralToInner(central) ; kırp
//     (v < 0 iken işaret v/w'den gelir: geri giderken sola dönüş için sağa kır)
//
// Dönüş: girdilerden herhangi biri sonlu değilse false döner ve *inner_out'a
// dokunmaz. Upstream sürücüde v == 0 ve w == 0 durumunda 0/0 = NaN üretilip
// int16'ya çevriliyordu (tanımsız davranış) — bu fonksiyon o durumda 0.0 verir.
bool twistToInnerSteering(
  double v, double w, double wheelbase, double track, double max_inner,
  double * inner_out);

// Ackermann odometrisi: iç tekerlek açısı ve doğrusal hızdan yaw hızı.
//   inner == 0 -> 0.0 ; aksi halde |w| = |v| / (L / tan|inner| + T/2),
//   işaret = sign(v) * sign(inner). Sonlu olmayan girdi -> 0.0
double ackermannYawRate(double v, double inner, double wheelbase, double track);

// Protokol komutu: değer * 1000, en yakın tamsayıya yuvarlanır ve int16
// aralığına [-32768, 32767] kırpılır. Sonlu olmayan değer -> 0.
// (Upstream `int16_t x = value * 1000;` aralık dışı ve NaN'de tanımsız davranıştı.)
int16_t toCommand(double value);

// Büyük endian 4 baytı işaretli int32'ye çevirir. Negatif değerler için de
// tanımlı davranış (upstream `data[0] << 24` int taşması yapıyordu).
int32_t decodeInt32BE(const uint8_t * data);

// IMU yaw [derece] için bir adımlık değişim:
//   delta = present - last, [-180, 180) aralığına sarılır (±180 geçişinde
//   -358 yerine +2 verir); |delta| < dead_band_deg ise 0.0.
// dead_band_deg <= 0 ise ölü bant yoktur. Sonlu olmayan girdi -> 0.0
double yawDeltaDeg(double present_deg, double last_deg, double dead_band_deg);

}  // namespace kinematics
}  // namespace AgileX

#endif  // LIMO_BASE__LIMO_KINEMATICS_HPP_
