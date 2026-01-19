# 🛰️ Lidar Verileri ile Hassas Yanaşma ve Geometrik Hesaplamalar

Bu proje, otonom mobil robotların (AMR) şarj ünitelerine veya belirlenmiş hedeflere milimetrik hassasiyetle yanaşabilmesi için geliştirilmiş, uçtan uca bir LIDAR veri işleme hattıdır. Sistem; ham veri okuma, filtreleme, çizgi tespiti ve geometrik analiz süreçlerini tamamen optimize edilmiş algoritmalarla gerçekleştirir.

---

## 🚀 Öne Çıkan Özellikler

*   **Özel TOML Ayrıştırıcı:** Herhangi bir dış kütüphane (nlohmann/json vb.) kullanmadan geliştirilmiş, yüksek performanslı manuel veri ayıklama sistemi.
*   **Dinamik Veri Yönetimi:** Sunum modunda `curl` entegrasyonu sayesinde uzak sunuculardan otomatik veri çekme yeteneği.
*   **Gelişmiş Görselleştirme:** Analiz sonuçlarının ve robot konumunun **Gnuplot** aracılığıyla dinamik grafiksel sunumu.
*   **Sıfır Bağımlılık:** Temel işlemler için harici kütüphane gerektirmez, taşınabilirliği yüksektir.

---

## 🧠 Kullanılan Algoritmalar ve Matematiksel Yöntemler

Proje, gürültülü sensör verisinden anlamlı geometrik çıkarımlar yapmak için üç aşamalı bir analiz süreci izler:

### 1. RANSAC (Random Sample Consensus)
Dağınık ve gürültülü LIDAR nokta bulutu içerisinden en tutarlı doğruları tespit etmek için kullanılır.
*   **Aykırı Değer Temizleme:** Rastgele örnekleme yaparak sensör gürültülerini (outliers) eler.
*   **Model Doğrulama:** En az 8 noktanın desteklediği modeller "geçerli doğru" kabul edilir.

### 2. En Küçük Kareler Yöntemi (Least Squares Refinement)
RANSAC ile bulunan aday doğrular, o doğruya ait tüm destekçi noktalar kullanılarak matematiksel olarak optimize edilir. Bu, robotun yanaşma hassasiyetini artırır.

### 3. Geometrik Analiz ve Kesişim Tespiti
*   **Koordinat Dönüşümü:** Kutupsal koordinatlar $(r, \theta)$, analiz için Kartezyen koordinatlara $(x, y)$ dönüştürülür.
*   **Kesişim Analizi:** Doğruların eğim ($m$) ve kayma ($n$) değerleri kullanılarak potansiyel hedef noktaları hesaplanır.
*   **Açısal Filtreleme:** Sadece $60^\circ$ ve üzeri kesişim açıları "geçerli hedef" olarak nitelendirilir.

---

## 💻 Programlama İlkeleri ve Mimari

Kod yapısı, sürdürülebilirlik ve performans odaklı şu prensipler üzerine kurulmuştur:

*   **Nesne Odaklı Yapılar:** `Nokta`, `Dogru` ve `KesisimNoktasi` gibi `struct` yapıları ile veri kapsüllenmiştir.
*   **Modüler Fonksiyon Yapısı:** Mesafe hesaplama, açı bulma ve kesişim tespiti bağımsız fonksiyonlara bölünmüştür.
*   **Hata Yönetimi:** `NaN` değerler, sensör limitleri dışındaki mesafeler ve dikey doğrular için özel kontrol mekanizmaları uygulanmıştır.
*   **Otomasyon:** Gnuplot betiklerinin (`.gp`) kod tarafında dinamik olarak oluşturulmasıyla uçtan uca analiz sağlanır.

---

## 🛠️ Gereksinimler

*   **Derleyici:** C++11 veya üzeri destekleyen bir derleyici (GCC, MSVC, Clang).
*   **Görselleştirme:** [Gnuplot](http://www.gnuplot.info/).
*   **Veri İndirme:** `curl` (Sunum modu için).

---

## 📈 Örnek Çıktı

Analiz sonucunda sistem şu çıktıları üretir:
1.  Robotun hedef noktasına olan net mesafesi.
2.  Kesişen doğruların indeksleri ve kesişim açısı.

---

## 👥 Geliştiriciler
*   Enes Canlı
