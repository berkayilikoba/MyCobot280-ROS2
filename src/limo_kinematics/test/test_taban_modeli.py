"""TabanModeli sözleşmesi: saf gecikme, birinci derece yaklaşma, ivme sınırı, kazanç, watchdog."""

import pytest

from limo_kinematics.taban_modeli import Kanal, KanalParam, TabanModeli


def calistir(k, komutlar, t_son, dt=0.01, tekrar_komut=True):
    """komutlar: [(t, u)]; tekrar_komut True ise her adımda son komut yeniden gönderilir (Nav2 gibi)."""
    out = []
    t = 0.0
    i = 0
    son = None
    while t <= t_son + 1e-9:
        while i < len(komutlar) and komutlar[i][0] <= t + 1e-9:
            son = komutlar[i][1]
            k.komut(t, son)
            i += 1
        if tekrar_komut and son is not None and i > 0:
            k.komut(t, son)
        out.append((t, k.adim(t)))
        t = round(t + dt, 6)
    return out


def deger(out, t):
    return min(out, key=lambda p: abs(p[0] - t))[1]


def test_saf_gecikme_suresince_hareket_yok():
    k = Kanal(KanalParam(gecikme=0.3, tau=0.05, ivme=100.0, kazanc=1.0, watchdog=10.0))
    out = calistir(k, [(0.0, 1.0)], 1.0)
    assert deger(out, 0.25) == pytest.approx(0.0, abs=1e-9)
    assert deger(out, 0.8) > 0.9


def test_birinci_derece_tau_aninda_yuzde63():
    k = Kanal(KanalParam(gecikme=0.0, tau=0.2, ivme=1e6, kazanc=1.0, watchdog=10.0))
    out = calistir(k, [(0.0, 1.0)], 1.0, dt=0.001)
    assert deger(out, 0.2) == pytest.approx(0.632, abs=0.02)


def test_ivme_siniri():
    k = Kanal(KanalParam(gecikme=0.0, tau=0.001, ivme=0.5, kazanc=1.0, watchdog=10.0))
    out = calistir(k, [(0.0, 1.0)], 1.0)
    assert deger(out, 0.5) == pytest.approx(0.25, abs=0.02)   # 0.5 m/s² x 0.5 s


def test_kazanc():
    k = Kanal(KanalParam(gecikme=0.0, tau=0.05, ivme=1e6, kazanc=1.05, watchdog=10.0))
    out = calistir(k, [(0.0, 0.4)], 2.0)
    assert deger(out, 2.0) == pytest.approx(0.42, abs=1e-3)


def test_watchdog_komut_kesilince_durur():
    k = Kanal(KanalParam(gecikme=0.0, tau=0.05, ivme=1e6, kazanc=1.0, watchdog=0.5))
    out = calistir(k, [(0.0, 1.0)], 2.0, tekrar_komut=False)   # tek komut, sonra sessizlik
    assert deger(out, 0.3) > 0.9
    assert deger(out, 1.5) == pytest.approx(0.0, abs=1e-3)


def test_watchdog_surekli_komutta_devreye_girmez():
    k = Kanal(KanalParam(gecikme=0.0, tau=0.05, ivme=1e6, kazanc=1.0, watchdog=0.5))
    out = calistir(k, [(0.0, 1.0)], 3.0, tekrar_komut=True)
    assert deger(out, 3.0) == pytest.approx(1.0, abs=1e-3)


def test_iki_kanal_bagimsiz():
    m = TabanModeli()
    v = w = 0.0
    t = 0.0
    while t < 3.0:
        m.komut(t, 0.1, 0.5)
        v, w = m.adim(t)
        t += 0.01
    assert v == pytest.approx(0.095, abs=0.005)   # kazanç 0.95
    assert w == pytest.approx(0.525, abs=0.01)    # kazanç 1.05
