# 🛰️ LIDAR Verisiyle Doğru ve Hedef Noktası Tespiti

2D LIDAR taramasından duvar benzeri doğruları bulan, bu doğruların **60° ve üzeri açıyla kesiştiği köşeleri** tespit eden ve robota en yakın köşeye olan mesafeyi hesaplayan bir C++ programı. Mobil bir robotun şarj istasyonu gibi bir hedefe yanaşması için gereken temel geometrik adımları uygular.

![Örnek çıktı: tespit edilen doğrular, hedef köşe ve robota mesafe](docs/grafik.png)

*Mavi noktalar geçerli LIDAR ölçümleri, yeşil çizgiler bulunan doğrular, sarı nokta seçilen hedef köşe, kırmızı kesikli çizgi robotla hedef arasındaki mesafe.*

---

## ⚙️ Nasıl Çalışır?

1. **Veri okuma:** TOML formatındaki tarama dosyasından açı/mesafe parametreleri ve `ranges` dizisi okunur. Bunun için harici bir kütüphane kullanılmaz, küçük bir elle yazılmış ayrıştırıcı vardır.
2. **Filtreleme ve dönüşüm:** `NaN` ve `range_min`/`range_max` dışındaki ölçümler elenir. Kalan kutupsal ölçümler $(r, \theta)$ kartezyen noktalara $(x, y)$ çevrilir.
3. **RANSAC ile doğru tespiti:** Rastgele seçilen nokta çiftlerinden aday doğrular üretilir. En çok noktanın 2 cm yakınından geçtiği doğru seçilir. En az 8 noktanın desteklediği doğrular kabul edilir, noktaları listeden çıkarılır ve işlem tekrarlanır.
4. **Parçalara ayırma:** Aynı doğrultuda olup arasında 1 m'den büyük boşluk bulunan noktalar ayrı parçalara bölünür. Böylece birbirinden uzak iki duvar tek doğru sayılmaz.
5. **En küçük kareler ile iyileştirme:** Her parçanın doğrusu, o parçadaki tüm noktalar kullanılarak yeniden hesaplanır.
6. **Kesişim analizi:** Doğru çiftlerinin kesişim noktaları ve aralarındaki açı bulunur. 60° ve üzeri kesişimler arasından robota (orijin) en yakın olanı hedef seçilir.
7. **Görselleştirme:** Sonuçlar `.dat` dosyalarına yazılır, bunları çizen bir Gnuplot betiği (`plot_script.gp`) üretilip açılır.

---

## 🛠️ Gereksinimler

- C++14 destekleyen bir derleyici (GCC, Clang veya MSVC)
- [Gnuplot](http://www.gnuplot.info/): grafik için. Kurulu değilse program yine çalışır, sadece grafiği açamaz.
- `curl`: yalnızca `--sunum` modu için

## ▶️ Derleme ve Çalıştırma

Komutları repo kök klasöründe çalıştırın:

```bash
g++ -std=c++14 -O2 -o lidar_dogru_tespit src/lidar_dogru_tespit.cpp

# Örnek veriyle çalıştır (ornek-veri/scan_data_NaN.toml)
./lidar_dogru_tespit

# Kendi tarama dosyanla çalıştır
./lidar_dogru_tespit tarama.toml

# Sunum modu: sunucudan rastgele bir tarama indirir
./lidar_dogru_tespit --sunum
```

Örnek veriyle beklenen sonuç:

```
Tespit Edilen Dogru Sayisi: 5
Robota en yakin 60+ derece hedef bulundu.
  Koordinat: (1.26925, 0.842981)
  Mesafe: 1.52368 metre
  Aci: 73.1526 derece
```

Test modunda rastgele sayı üreteci sabit bir tohumla (`srand(1)`) başlatılır, bu yüzden her çalıştırmada aynı sonuç alınır. Sunum modunda her çalıştırma farklı olabilir.

> **Not:** Sunum modunun veri çektiği üniversite sunucusu artık erişime kapalı (HTTP 403). İndirme başarısız olursa program otomatik olarak örnek veriyle devam eder.

---

## 📄 Girdi Formatı

```toml
[scan]
angle_min = 0.0              # radyan
angle_max = 4.71238898
angle_increment = 0.0174533
range_min = 0.2              # metre
range_max = 3.0
ranges = [ 1.0, 1.0, nan, 1.5, ... ]   # birden fazla satıra yayılabilir
```

---

## 🔧 Ayarlanabilir Parametreler

`src/lidar_dogru_tespit.cpp` içindeki sabitler:

| Sabit | Değer | Anlamı |
|---|---|---|
| `UZAKLIK_TOLERANSI` | 0.02 m | Noktanın doğruya ait sayılması için en fazla uzaklık |
| `MIN_DESTEKCI_SAYISI` | 8 | Bir doğrunun kabul edilmesi için gereken en az nokta |
| `ITERASYON_SAYISI` | 3000 | Her doğru için denenen rastgele aday sayısı |
| `MAX_NOKTA_ARASI_MESAFE` | 1.0 m | Aynı parçadaki ardışık iki nokta arası en fazla boşluk |
| `MIN_KESISIM_ACISI` | 60° | Hedef sayılması için en küçük kesişim açısı |
| `RANSAC_MIN_MESAFESI` | 0.5 m | Robota bundan yakın noktalar doğru aramaya katılmaz |

---

## 📁 Proje Yapısı

```
├── src/lidar_dogru_tespit.cpp   # Tüm program
├── ornek-veri/scan_data_NaN.toml # Örnek LIDAR taraması
└── docs/grafik.png               # README görseli
```

Program çalıştığı klasöre `points.dat`, `robot.dat`, `dogru_*.dat`, `hedef_nokta.dat`, `mesafe_cizgisi.dat` ve `plot_script.gp` dosyalarını yazar. Bu dosyalar `.gitignore` ile repoya eklenmez.

---

## ⚠️ Bilinen Sınırlamalar

- Kesişimler doğruların **sonsuz uzantıları** üzerinden hesaplanır. İki doğru parçası gerçekte birbirine uzak olsa bile uzantıları 60° ile kesişiyorsa hedef sayılabilir.
- `ranges` dizisindeki eleman sayısının `angle_min`, `angle_max` ve `angle_increment` ile tutarlı olup olmadığı kontrol edilmez.

---

## 👤 Geliştirici

Enes Canlı

## 📜 Lisans

[MIT](LICENSE)
