"""Gerçek LIMO tabanının (FOUR_DIFF) hız tepkisi — simülasyonda komutla hareket arasına konur.

Simde diff gövdesi planar_move ile komutu ANINDA uygular; gerçek taban ise gecikmeli ve ağır tepki verir.
Parametreler 24.09 teleop kaydından (saha/oturum_20260924_163343/bag) sistem tanımlamayla bulundu:
/cmd_vel -> [saf gecikme -> birinci derece gecikme (tau) -> ivme sınırı] x kazanç, ölçülen odom v / IMU w ile
en küçük hata (docs/sensor_modeli.md, "Taban modeli"). R²: v 0.86, w 0.71.

  v: gecikme 0.30 s, tau 0.45 s, ivme 0.15 m/s², kazanç 0.95
  w: gecikme 0.05 s, tau 0.20 s, ivme 6.0 rad/s², kazanç 1.05
  komut kesilirse (watchdog) hedef 0: v ~1.0 s, w ~0.5 s (en iyi uyum; donanımda ayrıca doğrulanmadı)
"""

import collections
from dataclasses import dataclass


@dataclass
class KanalParam:
    gecikme: float   # s, saf gecikme
    tau: float       # s, birinci derece zaman sabiti
    ivme: float      # birim/s², değişim hızı sınırı
    kazanc: float    # kararlı durum kazancı
    watchdog: float  # s, bu süre komut gelmezse hedef 0


V_VARSAYILAN = KanalParam(0.30, 0.45, 0.15, 0.95, 1.0)
W_VARSAYILAN = KanalParam(0.05, 0.20, 6.0, 1.05, 0.5)


class Kanal:
    """Tek eksen (v ya da w) için gecikme + birinci derece + ivme sınırı + kazanç."""

    def __init__(self, p: KanalParam):
        self.p = p
        self.komutlar = collections.deque()  # (zaman, değer)
        self.y = 0.0
        self.son_t = None

    def komut(self, t: float, u: float) -> None:
        self.komutlar.append((t, u))

    def _hedef(self, t: float) -> float:
        """t anında etkili komut: t - gecikme anına kadar gelmiş son komut; watchdog aşılmışsa 0."""
        te = t - self.p.gecikme
        son = None
        # t-gecikme'den eskileri at, en sonuncusunu tut
        while self.komutlar and self.komutlar[0][0] <= te:
            son = self.komutlar.popleft()
        if son is not None:
            self._etkin = son
        etkin = getattr(self, '_etkin', None)
        if etkin is None or te - etkin[0] > self.p.watchdog:
            return 0.0
        return self.p.kazanc * etkin[1]

    def adim(self, t: float) -> float:
        hedef = self._hedef(t)
        if self.son_t is None:
            self.son_t = t
            return self.y
        dt = max(0.0, t - self.son_t)
        self.son_t = t
        if dt == 0.0:
            return self.y
        a = dt / (self.p.tau + dt) if self.p.tau > 0 else 1.0
        istenen = self.y + a * (hedef - self.y)
        adim_max = self.p.ivme * dt
        self.y += max(-adim_max, min(adim_max, istenen - self.y))
        return self.y


class TabanModeli:
    def __init__(self, v: KanalParam = V_VARSAYILAN, w: KanalParam = W_VARSAYILAN):
        self.v = Kanal(v)
        self.w = Kanal(w)

    def komut(self, t: float, v: float, w: float) -> None:
        self.v.komut(t, v)
        self.w.komut(t, w)

    def adim(self, t: float):
        return self.v.adim(t), self.w.adim(t)
