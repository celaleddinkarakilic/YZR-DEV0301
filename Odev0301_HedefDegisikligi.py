# ODEV 301 Celaleddin Karakilic

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

### Robot Parametreleri ###
m = 5      # kütle
b = 2       # sürtünme katsayısı
v_i = 0     # ilk hız

### PID Katsayıları ###
K_p = 25 # Oransal Katsayı
K_i = 15  # Integral Katsayısı
K_d = 1  # Türev Katsayı

### Kontrol Hedefi ###
v_hedef = 1

# Bozucu parametreler
t_degisiklik= 5        # Değişiklik baş T

def Newton(y, t):
    v = y[0]            #Anlık Hız
    int_e = y[1]        #Hata Integral
    if t < t_degisiklik:
        v_hedef_anlik = 1.0
    else:
        v_hedef_anlik = 0.5

    hata = v_hedef_anlik - v
    d_hata = -v

    F_motor = (K_p*hata) + (K_i*int_e) + (K_d*d_hata)


    dvdt = (F_motor - b * v) /m
    d_int_dt = hata     #integral hata

    return [dvdt, d_int_dt]


### Simülasyon Zaman Ayarları ###
t_ilk = 0       #t_ilk
t_son = 10     #t_son
t_adım = 1000   #t_ilk ve t_son arası adım sayısı
t = np.linspace(t_ilk, t_son, t_adım)
Hiz = [v_i, 0]  #Başta hız=0, integral hata=0

### Çözüm ###
sonuc = odeint(Newton, Hiz, t)
v_sonuc = sonuc[:, 0]


### Tablo Oluşturma ###
plt.figure(figsize=(12,7))
plt.plot(t, v_sonuc, label='Hız $v(t)$', color='black')
plt.plot(t, [1.0 if ti<t_degisiklik else 0.5 for ti in t], 'orange', linestyle='--', label='Hedef Hız $v_{ref}(t)$')
plt.xlabel('Zaman (s)')
plt.ylabel('Hız (m/s)')
plt.title('ODEV 301 V-t')
plt.grid(True)
plt.legend()
plt.show()