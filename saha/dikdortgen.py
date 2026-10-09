#!/usr/bin/env python3
"""Dikdörtgen tur (PC, Humble): başlangıç pozundan kenar boyunca düz git, köşede yerinde sola dön.

Konum: TF map->base_link (AMCL + odom_duz). Komut: cmd_vel_smoothed -> collision_monitor -> cmd_vel,
yani Nav2'nin çarpışma güvenliği korunur (Nav2 açık olmalı: saha/nav_baslat.sh).
Dikdörtgen, betik başladığı andaki pozdan tanımlanır (köşe 0 = o an robotun durduğu yer, ilk kenar = baktığı yön).

Kullanım:
  dikdortgen.py [--kenarlar 2,1,2,1] [--tur 1] [--hiz 0.15] [--donus 0.4] [--sag]
Çıktı: her köşede varış hatası (kenar boyu / yanal / yön) ekrana ve --log dosyasına (CSV).
"""

import argparse
import math
import threading
import time

import rclpy
import rclpy.executors
import rclpy.parameter
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Dikdortgen(Node):

    def __init__(self, a):
        super().__init__('dikdortgen', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, a.sim)])
        self.a = a
        self.pub = self.create_publisher(Twist, a.cmd_topic, 10)
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)

    def poz(self):
        """(x, y, yaw) --cerceve çerçevesinde (map: AMCL, odom: odom_duz); dönüşüm 0.5 s'den eskiyse None."""
        try:
            tr = self.tfb.lookup_transform(self.a.cerceve, 'base_link', Time())
        except Exception:
            return None
        yas = (self.get_clock().now() - Time.from_msg(tr.header.stamp)).nanoseconds * 1e-9
        if yas > 0.5:
            return None
        q = tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return tr.transform.translation.x, tr.transform.translation.y, yaw

    def gonder(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.pub.publish(m)

    def bekle_poz(self):
        while rclpy.ok():
            time.sleep(0.05)
            p = self.poz()
            if p is not None:
                return p
        return None

    def dur(self, sure=0.6):
        t0 = time.time()
        while time.time() - t0 < sure and rclpy.ok():
            self.gonder(0.0, 0.0)
            time.sleep(0.05)

    def adim(self):
        """Bir kontrol adımı: poz al (TF bayatsa dur ve bekle)."""
        time.sleep(1.0 / self.a.hz)  # duvar saati; sim gerçek zamanlı (RTF~1) koştuğu için yeterli
        p = self.poz()
        if p is None:
            self.gonder(0.0, 0.0)
            self.get_logger().warn(f'{self.a.cerceve}->base_link bayat/yok: duruyorum', throttle_duration_sec=2.0)
        return p

    def kenar(self, x0, y0, yon, boy):
        """(x0,y0)'dan yon doğrultusunda boy kadar: hat üzerinde kal, sona yavaşlayarak dur."""
        c, s = math.cos(yon), math.sin(yon)
        while rclpy.ok():
            p = self.adim()
            if p is None:
                continue
            dx, dy = p[0] - x0, p[1] - y0
            ilerleme = c * dx + s * dy          # kenar boyunca
            yanal = -s * dx + c * dy            # sola +
            kalan = boy - ilerleme
            if kalan <= self.a.tol:
                break
            # hatta dönüş: yanal hatayı ve yön hatasını düzelt (pure pursuit benzeri)
            hedef_yon = yon + math.atan2(-yanal, max(self.a.bakis, 0.05))
            e = wrap(hedef_yon - p[2])
            v = min(self.a.hiz, self.a.yavas_k * kalan + self.a.v_min)
            if abs(e) > math.radians(25):        # çok saptıysa önce yönü düzelt
                v = 0.0
            w = max(-self.a.donus, min(self.a.donus, self.a.k_yon * e))
            self.gonder(v, w)
        self.dur()

    def don(self, hedef_yon):
        """Yerinde dön; sürtünmeyi yenmek için en az w_min."""
        while rclpy.ok():
            p = self.adim()
            if p is None:
                continue
            e = wrap(hedef_yon - p[2])
            if abs(e) < math.radians(self.a.yon_tol):
                break
            w = self.a.k_don * e
            w = math.copysign(min(self.a.donus, max(self.a.w_min, abs(w))), w)
            self.gonder(0.0, w)
        self.dur()

    def calistir(self):
        p0 = self.bekle_poz()
        if p0 is None:
            return
        x, y, yon = p0
        kenarlar = [float(k) for k in self.a.kenarlar.split(',')]
        isaret = -1.0 if self.a.sag else 1.0
        self.get_logger().info(f'başlangıç {self.a.cerceve}=({x:.3f},{y:.3f}) yön={math.degrees(yon):.1f}°, '
                               f'kenarlar={kenarlar}, tur={self.a.tur}')
        log = open(self.a.log, 'a') if self.a.log else None
        if log:
            log.write('t,tur,kose,hedef_x,hedef_y,hedef_yon_deg,x,y,yon_deg,boy_hata_m,yanal_hata_m,yon_hata_deg\n')
        for tur in range(1, self.a.tur + 1):
            for i, boy in enumerate(kenarlar):
                self.kenar(x, y, yon, boy)
                hx, hy = x + boy * math.cos(yon), y + boy * math.sin(yon)
                yon = wrap(yon + isaret * math.pi / 2)
                self.don(yon)
                p = self.bekle_poz()
                c, s = math.cos(yon), math.sin(yon)
                dx, dy = p[0] - hx, p[1] - hy
                satir = (time.time(), tur, i + 1, hx, hy, math.degrees(yon), p[0], p[1], math.degrees(p[2]),
                         c * dx + s * dy, -s * dx + c * dy, math.degrees(wrap(p[2] - yon)))
                self.get_logger().info(f'tur {tur} köşe {i + 1}: konum hatası '
                                       f'{math.hypot(dx, dy) * 100:.1f} cm, yön hatası {satir[-1]:+.1f}°')
                if log:
                    log.write(','.join(f'{v:.4f}' if isinstance(v, float) else str(v) for v in satir) + '\n')
                    log.flush()
                x, y = hx, hy              # bir sonraki kenar ideal köşeden başlar (hata birikmez)
        self.dur(1.0)
        self.get_logger().info('tur bitti')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--kenarlar', default='2,1,2,1')
    ap.add_argument('--tur', type=int, default=1)
    ap.add_argument('--hiz', type=float, default=0.15, help='m/s')
    ap.add_argument('--donus', type=float, default=0.4, help='rad/s (en çok)')
    ap.add_argument('--w_min', type=float, default=0.12, help='yerinde dönüşte en az rad/s')
    ap.add_argument('--sag', action='store_true', help='sağa dön (varsayılan sol)')
    ap.add_argument('--tol', type=float, default=0.01, help='kenar sonu, m')
    ap.add_argument('--yon_tol', type=float, default=1.5, help='dönüş, derece')
    ap.add_argument('--bakis', type=float, default=0.4, help='yanal düzeltme bakış mesafesi, m')
    ap.add_argument('--k_yon', type=float, default=1.5)
    ap.add_argument('--k_don', type=float, default=1.2)
    ap.add_argument('--yavas_k', type=float, default=0.5, help='kenar sonu yavaşlama, 1/s')
    ap.add_argument('--v_min', type=float, default=0.03)
    ap.add_argument('--hz', type=float, default=20.0)
    ap.add_argument('--cmd_topic', default='cmd_vel_smoothed')
    ap.add_argument('--log', default='')
    ap.add_argument('--sim', action='store_true', help='sim saati (use_sim_time)')
    ap.add_argument('--cerceve', default='map', choices=['map', 'odom'],
                    help='konum kaynağı. odom önerilir: 09.10 turunda AMCL yerinde dönüşte 5-32 cm sıçradı, '
                         'odom (IMU yönlü) ICP\'ye göre dönüşte <=1°, yer değiştirmede 1-8 cm')
    a = ap.parse_args()
    rclpy.init()
    n = Dikdortgen(a)
    # TF ve /clock aboneliklerini ayrı iş parçacığı işler; kontrol döngüsü yalnızca uyur ve okur.
    # (spin_once her çağrıda tek mesaj işliyordu: yüzlerce TF/s gerisinde kalıp pozu hep bayat gösteriyordu.)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        n.calistir()
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(10):
            if not rclpy.ok():
                break
            n.gonder(0.0, 0.0)
            time.sleep(0.05)
        ex.shutdown()          # iş parçacığı spin'deyken düğümü yok etmek C++ tarafında terminate'e yol açıyordu
        th.join(timeout=2.0)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
