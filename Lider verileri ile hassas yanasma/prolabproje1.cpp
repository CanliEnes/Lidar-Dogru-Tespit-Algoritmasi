#include <iostream>
#include <sstream>      // String'leri ayrıştırmak için
#include <string>
#include <fstream>
#include <vector>       // Sayıları saklamak için vector
#include <algorithm>    // sort, replace
#include <cmath>        // 'nan', sqrt, pow, abs, atan
#include <cstdlib>      // system(), rand(), srand()
#include <ctime>        // time(NULL)

using namespace std;

// (Derleme hatası almamak için)
const double PI = 3.14159265358979323846;

// --- YAPI TANIMLAMALARI  ---

struct Nokta {
    double x = 0.0;
    double y = 0.0;
};


struct Dogru {
    vector<Nokta> noktalar;
    double m = 0.0; // Eğim
    double c = 0.0; // Sabit
    bool dikey_mi = false;
    double x_kesme_noktasi = 0.0;
};


struct KesisimNoktasi {
    Nokta p;
    double aci = 0.0;
    double robota_mesafe = 0.0;
    size_t dogru_index1 = 0;
    size_t dogru_index2 = 0;
};


// --- RANSAC YARDIMCI FONKSİYONLARI ---

double mesafeHesapla(Nokta p1, Nokta p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double noktadanDogruyaUzaklik(Nokta p, Dogru d) {
    if (d.dikey_mi) {
        return abs(p.x - d.x_kesme_noktasi);
    }
    else {
        if (isnan(d.m)) return INFINITY; // Veya başka bir mantıklı değer
        return abs(d.m * p.x - p.y + d.c) / sqrt(pow(d.m, 2) + 1); //abs islem sonucunu pozitif yapar
    }
}

// --- ANALİZ FONKSİYONLARI  ---

double ikiDogruAci(Dogru d1, Dogru d2) {
    if (d1.dikey_mi && d2.dikey_mi) {
        return 0.0; // Paraleller
    }
    if (d1.dikey_mi && !isnan(d2.m)) {
        return 90.0 - abs(atan(d2.m) * 180.0 / PI);
    }
    if (d2.dikey_mi && !isnan(d1.m)) {
        return 90.0 - abs(atan(d1.m) * 180.0 / PI);
    }
    if (!isnan(d1.m) && !isnan(d2.m)) {
        if (abs(1.0 + d1.m * d2.m) < 0.0001) { return 90.0; } // Dik kesişim
        // Paydanın sıfır olmamasını sağlamak için ek kontrol (çok küçük eğimler için nadir)
        if (abs(1.0 + d1.m * d2.m) < 1e-9) return 90.0; // Pratik olarak dik
        double tan_theta = abs((d1.m - d2.m) / (1.0 + d1.m * d2.m));
        return atan(tan_theta) * 180.0 / PI;
    }
    return 0.0; // Beklenmedik durum
}

Nokta ikiDogruKesisim(Dogru d1, Dogru d2, bool& bulundu) {
    Nokta kesisim_p = { 0.0, 0.0 }; // C4700 HATASI ÇÖZÜMÜ!!!!!!!!!!!!!!!!!
    bulundu = false;
    if (d1.dikey_mi && d2.dikey_mi) {
        bulundu = false; // Paralel
        return kesisim_p;
    }

    if (d1.dikey_mi) {
        if (isnan(d2.m)) return kesisim_p;// Eğer d2 de Nan ise hesaplama yapılamaz
        kesisim_p.x = d1.x_kesme_noktasi;
        kesisim_p.y = d2.m * kesisim_p.x + d2.c;
        bulundu = true;
        return kesisim_p;
    }

    if (d2.dikey_mi) {
        if (isnan(d1.m)) return kesisim_p;// Eğer d1 de Nan ise hesaplama yapılamaz
        kesisim_p.x = d2.x_kesme_noktasi;
        kesisim_p.y = d1.m * kesisim_p.x + d1.c;
        bulundu = true;
        return kesisim_p;
    }

    if (isnan(d1.m) || isnan(d2.m) || abs(d1.m - d2.m) < 0.0001) {
        bulundu = false; // Paralel veya geçersiz eğim
        return kesisim_p;
    }

    // Paydanın sıfır olmamasını sağlama (pratik olarak)
    if (abs(d1.m - d2.m) < 1e-9) {
        bulundu = false;
        return kesisim_p;
    }

    kesisim_p.x = (d2.c - d1.c) / (d1.m - d2.m);
    kesisim_p.y = d1.m * kesisim_p.x + d1.c;
    bulundu = true;
    return kesisim_p;
}

// En Küçük Kareler Yöntemi ile Doğru Tespiti verilen bir nokta kümesine en uygun doğruyu hesaplar.
Dogru dogruyaEnIyiUyanCizgiyiBul(const vector<Nokta>& noktalar) {
    Dogru sonuc;
    if (noktalar.size() < 2) {
        // Yeterli nokta yoksa geçersiz bir doğru döndür
        sonuc.m = NAN;
        sonuc.c = NAN;
        return sonuc;
    }

    double toplam_x = 0.0, toplam_y = 0.0, toplam_xy = 0.0, toplam_x2 = 0.0;
    double min_x = noktalar[0].x, max_x = noktalar[0].x;

    for (const auto& p : noktalar) {
        toplam_x += p.x;
        toplam_y += p.y;
        toplam_xy += p.x * p.y;
        toplam_x2 += p.x * p.x;
        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;
    }

    double n = (double)noktalar.size();

    // Dikey doğru kontrolü: Eğer x'deki değişim çok azsa, dikey kabul et
    if (abs(max_x - min_x) < 0.01) { // 1cm'den az x değişimi
        sonuc.dikey_mi = true;
        sonuc.x_kesme_noktasi = toplam_x / n; // Tüm x'lerin ortalaması
        sonuc.m = NAN;
        sonuc.c = NAN;
    } else {
        // Normal eğim hesaplaması
        double payda = n * toplam_x2 - toplam_x * toplam_x;
        if (abs(payda) < 1e-9) { // Sıfıra bölme hatasını önle
             // Bu durumun yaşanmaması gerekir ama önlem olarak dikey kabul edelim
            sonuc.dikey_mi = true;
            sonuc.x_kesme_noktasi = toplam_x / n;
            sonuc.m = NAN;
            sonuc.c = NAN;
        } else {
            sonuc.dikey_mi = false;
            sonuc.m = (n * toplam_xy - toplam_x * toplam_y) / payda;
            sonuc.c = (toplam_y - sonuc.m * toplam_x) / n;
            sonuc.x_kesme_noktasi = NAN;
        }
    }
    
    sonuc.noktalar = noktalar; 
    return sonuc;
}


// --- ANA FONKSİYON ---

int main()
{
    // --- MOD SEÇİMİ ---
    bool test_modu = true; // Test modu için 'true', Sunum modu için 'false' yap

    if (test_modu) {
        srand(1); 
    }
    else {
        srand(time(NULL)); // sunum modu için her seferinde farklı rastgelelik
    }

    string kullanilacak_dosya_adi; 

    if (test_modu) {
        // --- YEREL DOSYA OKUMA ---
        string yerel_test_dosyasi = "C:\\Users\\canli\\Downloads\\scan_data_NaN.toml";
        cout << "--- TEST MODU AKTIF ---" << endl;
        cout << "Yerel test dosyasi kullaniliyor: " << yerel_test_dosyasi << endl;
        kullanilacak_dosya_adi = yerel_test_dosyasi; // Test modunda doğrudan bu dosyayı kullan
    }
    else {
        // --- INTERNETTEN INDIRME ---
        int dosya_numarasi = (rand() % 5) + 1; 
        string base_url = "http://abilgisayar.kocaeli.edu.tr/lidar";
        string file_extension = ".toml";
        string url = "base_url + to_string(dosya_numarasi) + file_extension";
        string indirilen_dosya_adi = "indirilen_scan.toml"; 
        string download_command = "curl -o " + indirilen_dosya_adi + " " + url;

        cout << "--- SUNUM MODU AKTIF ---" << endl;
        cout << "Rastgele secilen dosya: lidar" << dosya_numarasi << ".toml" << endl;
        cout << "Dosya indiriliyor: " << url << endl;
        int result = system(download_command.c_str());
        if (result != 0) {
            cout << "Dosya indirme basarisiz! 'curl' kurulu ve internet baglantisi oldugundan emin olun." << endl;
            cout << "Indirme basarisiz, yerel test dosyasi kullanilacak..." << endl;
            kullanilacak_dosya_adi = "C:\\Users\\canli\\Downloads\\scan_data_NaN.toml"; // Hata durumunda test dosyasını kullan güvenlik içindir
        }
        else {
            cout << "Dosya basariyla indirildi: " << indirilen_dosya_adi << endl;
            kullanilacak_dosya_adi = indirilen_dosya_adi; 
        }
    }
    cout << "------------------------" << endl;

    // --- DOSYA OKUMA VE AYRIŞTIRMA ---

    double anglemin = 0.0, anglemax = 0.0, angleincrement = 0.0;
    double rangemin = 0.0, rangemax = 0.0;

    vector<double> ranges_data;

    ifstream dosya;
    dosya.open(kullanilacak_dosya_adi);
    if (dosya.is_open()) {
        cout << "Dosya acildi." << endl;
        string kelime;
        while (getline(dosya, kelime)) {
            if (kelime.find("angle_min") != string::npos) {
                anglemin = stod(kelime.substr(kelime.find("=") + 1));
            }
            else if (kelime.find("angle_max") != string::npos) {
                anglemax = stod(kelime.substr(kelime.find("=") + 1));
            }
            else if (kelime.find("angle_increment") != string::npos) {
                angleincrement = stod(kelime.substr(kelime.find("=") + 1));
            }
            else if (kelime.find("range_min") != string::npos) {
                rangemin = stod(kelime.substr(kelime.find("=") + 1));
            }
            else if (kelime.find("range_max") != string::npos) {
                rangemax = stod(kelime.substr(kelime.find("=") + 1));
            }
            else if (kelime.find("ranges") != string::npos) {
                string current_line = kelime;
                bool in_array = true;
                while (in_array && dosya) {
                    size_t start_pos = current_line.find("[");
                    if (start_pos != string::npos) {
                        current_line = current_line.substr(start_pos + 1);
                    }
                    size_t end_pos = current_line.find("]");
                    if (end_pos != string::npos) {
                        current_line = current_line.substr(0, end_pos);
                        in_array = false;
                    }
                    replace(current_line.begin(), current_line.end(), ',', ' ');
                    stringstream ss(current_line);
                    string temp_val;
                    while (ss >> temp_val) {
                        if (temp_val == "'nan'" || temp_val == "nan") {
                            ranges_data.push_back(NAN);
                        }
                        else {
                            try {
                                ranges_data.push_back(stod(temp_val));
                            }
                            catch (const std::invalid_argument&) {} // 'e' değişkeni kaldırıldı
                        }
                    }
                    if (in_array) {
                        getline(dosya, current_line);
                    }
                }
            }
        }
        dosya.close();
    }
    else {
        cout << "Kullanilacak dosya '" << kullanilacak_dosya_adi << "' acilamadi!" << endl;
        return 1;
    }

    cout << "------------------------" << endl;
    cout << "angle_min: " << anglemin << endl;
    cout << "angle_max: " << anglemax << endl;
    cout << "range_min: " << rangemin << endl;
    cout << "range_max: " << rangemax << endl;
    cout << "Okunan 'ranges' eleman sayisi: " << ranges_data.size() << endl;
    // --- NOKTALARI FİLTRELEME VE DÖNÜŞÜM ---

    ofstream robot_file("robot.dat");
    robot_file << "0 0\n";
    robot_file.close();

    ofstream points_file("points.dat");
    vector<Nokta> gecerli_noktalar; // RANSAC için
    int nantutucu = 0;
    int toplam_gecerli_nokta_sayaci = 0; // Görsel için

    int num_points = (int)ranges_data.size(); // (size_t -> int dönüşümü)
    const double RANSAC_MIN_MESAFESI = 0.5; // 50cm

    for (int i = 0; i < num_points; ++i) {
        if (isnan(ranges_data[i]) || ranges_data[i] < rangemin || ranges_data[i] > rangemax) {
            nantutucu++;
            continue;
        }
        else {
            toplam_gecerli_nokta_sayaci++;
            double angle = anglemin + i * angleincrement;
            double x = ranges_data[i] * cos(angle);
            double y = ranges_data[i] * sin(angle);
            points_file << x << " " << y << "\n";
            if (ranges_data[i] > RANSAC_MIN_MESAFESI) {
                Nokta p;
                p.x = x;
                p.y = y;
                gecerli_noktalar.push_back(p);
            }
        }
    }
    points_file.close();
    cout << "Filtrelenen (gecersiz/NaN) nokta sayisi: " << nantutucu << endl;
    cout << "Gecerli nokta sayisi (Gorsel icin): " << toplam_gecerli_nokta_sayaci << endl;
    cout << "Gecerli nokta sayisi (RANSAC icin): " << gecerli_noktalar.size() << " (50cm filtresi uygulandi)" << endl;
    
    // --- DOĞRU TESPİTİ: RANSAC ---

    const double UZAKLIK_TOLERANSI = 0.02; // 6cm (noktanın doğruya uzaklığı)
    const int MIN_DESTEKCI_SAYISI = 8;
    const int ITERASYON_SAYISI = 3000;
    const double MAX_NOKTA_ARASI_MESAFE = 1; // Bir doğru üzerindeki iki nokta arası max mesafe (12cm)

    vector<Dogru> tespitEdilenDogrular;
    vector<Nokta> kalan_noktalar = gecerli_noktalar;

    while (kalan_noktalar.size() >= MIN_DESTEKCI_SAYISI && tespitEdilenDogrular.size() < 10) {
        Dogru en_iyi_dogru;
        int en_iyi_destekci_sayisi = 0;
        vector<Nokta> en_iyi_destekciler;
        if (kalan_noktalar.size() < 2) break; // 2'den az nokta kaldıysa çık

        for (int i = 0; i < ITERASYON_SAYISI; ++i)
        {
            int index1 = rand() % (int)kalan_noktalar.size();
            int index2 = rand() % (int)kalan_noktalar.size();
            if (index1 == index2) continue;
            Nokta p1 = kalan_noktalar[index1];
            Nokta p2 = kalan_noktalar[index2];
            Dogru aday_dogru;
            if (abs(p1.x - p2.x) < 0.0001) {
                aday_dogru.dikey_mi = true;
                aday_dogru.x_kesme_noktasi = p1.x;
                aday_dogru.m = NAN;
                aday_dogru.c = NAN;
            }
            else {
                aday_dogru.dikey_mi = false;
                aday_dogru.m = (p2.y - p1.y) / (p2.x - p1.x);
                aday_dogru.c = p1.y - aday_dogru.m * p1.x;
                aday_dogru.x_kesme_noktasi = NAN;
            }

            int guncel_destekci_sayisi = 0;
            vector<Nokta> guncel_destekciler;
            for (const Nokta& p : kalan_noktalar) {
                if (noktadanDogruyaUzaklik(p, aday_dogru) < UZAKLIK_TOLERANSI) {
                    guncel_destekci_sayisi++;
                    guncel_destekciler.push_back(p);
                }
            }

            if (guncel_destekci_sayisi > en_iyi_destekci_sayisi) {
                en_iyi_destekci_sayisi = guncel_destekci_sayisi;
                en_iyi_dogru = aday_dogru;
                en_iyi_destekciler = guncel_destekciler;
            }
        }

        // AMAÇ: Bulunan destekçi noktalarını, aralarındaki mesafeye göre sürekli parçalara ayırmak.
        if (en_iyi_destekci_sayisi >= MIN_DESTEKCI_SAYISI) {
            
            // EN KÜÇÜK KARELER İLE DOĞRUYU RAFİNE ETME KISMI
            // Rastgele iki noktadan oluşturulan model yerine, tüm destekçi noktalara uyan
            // en iyi doğruyu (line of best fit) hesaplıyoruz.
            Dogru rafine_edilmis_dogru = dogruyaEnIyiUyanCizgiyiBul(en_iyi_destekciler);

            // 1. ADIM: Destekçi noktaları YENİ ve DOĞRU model üzerinde sırala.
            sort(en_iyi_destekciler.begin(), en_iyi_destekciler.end(), [&](const Nokta& a, const Nokta& b) {
                if (rafine_edilmis_dogru.dikey_mi) { // Artık rafine edilmiş doğruyu kullanıyoruz
                    return a.y < b.y; // Dikey doğru ise y eksenine göre sırala
                }
                return a.x < b.x; // Normal doğruları x eksenine göre sırala
            });

            // 2. ADIM: Sıralanmış noktaları sürekli parçalara (segmentlere) ayır.
            vector<vector<Nokta>> dogru_parcalari;
            if (!en_iyi_destekciler.empty()) {
                dogru_parcalari.push_back({ en_iyi_destekciler[0] }); // İlk parçayı ilk noktayla başlat

                for (size_t k = 1; k < en_iyi_destekciler.size(); ++k) {
                    double aradaki_mesafe = mesafeHesapla(en_iyi_destekciler[k], en_iyi_destekciler[k - 1]);
                    
                    if (aradaki_mesafe <= MAX_NOKTA_ARASI_MESAFE) {
                        dogru_parcalari.back().push_back(en_iyi_destekciler[k]);
                    } else {
                        dogru_parcalari.push_back({ en_iyi_destekciler[k] });
                    }
                }
            }
            
            // 3. ADIM: Yeterli sayıda noktaya sahip olan parçaları geçerli doğru olarak kabul et.
            vector<Nokta> tum_kullanilan_noktalar;
            for (const auto& parca : dogru_parcalari) {
                if (parca.size() >= MIN_DESTEKCI_SAYISI) {
                    // Her bir geçerli parça için, sadece o parçanın noktalarını kullanarak
                    // son bir kez daha en uygun doğruyu hesapla. Bu, en yüksek hassasiyeti sağlar.
                    Dogru son_dogru_modeli = dogruyaEnIyiUyanCizgiyiBul(parca);
                    tespitEdilenDogrular.push_back(son_dogru_modeli);

                    // Bu parçadaki noktaları, bir sonraki RANSAC turundan çıkarmak için birleştir
                    tum_kullanilan_noktalar.insert(tum_kullanilan_noktalar.end(), parca.begin(), parca.end());
                }
            }

            // 4. ADIM: Kabul edilen tüm parçaların noktalarını "kalan_noktalar" listesinden çıkar.
            if (!tum_kullanilan_noktalar.empty()) {
                vector<Nokta> sonraki_tur_noktalari;
                for (const Nokta& p_kalan : kalan_noktalar) {
                    bool bulundu = false;
                    for (const Nokta& p_kullanilan : tum_kullanilan_noktalar) {
                        if (abs(p_kalan.x - p_kullanilan.x) < 0.0001 && abs(p_kalan.y - p_kullanilan.y) < 0.0001) {
                            bulundu = true;
                            break;
                        }
                    }
                    if (!bulundu) {
                        sonraki_tur_noktalari.push_back(p_kalan);
                    }
                }
                kalan_noktalar = sonraki_tur_noktalari;
            } else {
                break;
            }

        } else {
            // Yeterli destekçi bulunamadıysa, daha fazla doğru aramanın anlamı yok.
            break; 
        }
    }

    // --- RANSAC BÖLÜMÜ SONU ---

    cout << "------------------------" << endl;
    cout << "RANSAC TAMAMLANDI." << endl;
    cout << "Tespit Edilen Dogru Sayisi: " << tespitEdilenDogrular.size() << endl;
    for (size_t i = 0; i < tespitEdilenDogrular.size(); ++i) {
        cout << "  Dogru " << i + 1 << ": " << tespitEdilenDogrular[i].noktalar.size() << " adet noktaya sahip." << endl;
    }
    cout << "------------------------" << endl;
    // --- GEOMETRİK ANALİZ ---

    vector<KesisimNoktasi> onemliKesisimler;
    const double MIN_KESISIM_ACISI = 60.0;
    cout << "GEOMETRIK ANALIZ BASLADI..." << endl;

    for (size_t i = 0; i < tespitEdilenDogrular.size(); ++i) {
        for (size_t j = i + 1; j < tespitEdilenDogrular.size(); ++j) {

            Dogru d1 = tespitEdilenDogrular[i];
            Dogru d2 = tespitEdilenDogrular[j];

            bool kesisim_var_mi = false;
            Nokta kesisim_p = ikiDogruKesisim(d1, d2, kesisim_var_mi);
            if (kesisim_var_mi) {
                double aci = ikiDogruAci(d1, d2);
                cout << "   - Dogru " << i + 1 << " ve Dogru " << j + 1 << " kesisiyor. Aci: " << aci << " derece." << endl;

                if (aci >= MIN_KESISIM_ACISI) {
                    Nokta robot_konumu = { 0.0, 0.0 };
                    double mesafe = mesafeHesapla(robot_konumu, kesisim_p);

                    KesisimNoktasi yeni_onemli_nokta;
                    yeni_onemli_nokta.p = kesisim_p;
                    yeni_onemli_nokta.aci = aci;
                    yeni_onemli_nokta.robota_mesafe = mesafe;
                    yeni_onemli_nokta.dogru_index1 = i;
                    yeni_onemli_nokta.dogru_index2 = j;


                    onemliKesisimler.push_back(yeni_onemli_nokta);
                }
            }
        }
    }

    cout << "------------------------" << endl;
    cout << "ANALIZ TAMAMLANDI." << endl;
    cout << "Toplam 60+ derece Kesisim Sayisi: " << onemliKesisimler.size() << endl;

    KesisimNoktasi en_yakin_hedef;
    bool hedef_bulundu = false;

    if (!onemliKesisimler.empty()) {
        hedef_bulundu = true;
        en_yakin_hedef = onemliKesisimler[0];
        for (const KesisimNoktasi& k : onemliKesisimler) {
            if (k.robota_mesafe < en_yakin_hedef.robota_mesafe) {
                en_yakin_hedef = k;
            }
        }
        cout << "Robota en yakin 60+ derece hedef bulundu." << endl;
        cout << "  Koordinat: (" << en_yakin_hedef.p.x << ", " << en_yakin_hedef.p.y << ")" << endl;
        cout << "  Mesafe: " << en_yakin_hedef.robota_mesafe << " metre" << endl;
        cout << "  Aci: " << en_yakin_hedef.aci << " derece" << endl;
    }
    else {
        cout << "Uygun (60+ derece) kesisim noktasi bulunamadi." << endl;
    }
    cout << "------------------------" << endl;
    // --- GEOMETRİK ANALİZ BÖLÜMÜ SONU ---


    // --- GNUPLOT SCRIPT OLUŞTURMA ---

    ofstream script_file("plot_script.gp");
    if (!script_file.is_open()) {
        std::cerr << "plot_script.gp dosyasi olusturulamadi!" << std::endl;
        return 1;
    }
    
    script_file << "# --- Genel Ayarlar ---\n";
    script_file << "set terminal wxt enhanced\n";
    script_file << "set title 'Lidar Verisi (Hedef Tespiti ile)'\n";
    script_file << "set xlabel 'X Ekseni (m)'\n";
    script_file << "set ylabel 'Y Ekseni (m)'\n";
    script_file << "set grid\n";
    script_file << "set size square\n";
    script_file << "set xrange [-3:3]\n";
    script_file << "set yrange [-3:3]\n";
    script_file << "set key right top outside\n";
    script_file << "set key box linetype 1 linewidth 1.5\n";
    script_file << "set key title 'Gosterim'\n";
    script_file << "set key spacing 1.2\n";
    script_file << "set key samplen 2\n";
    
    // --- Hedef Noktayı ve Mesafeyi Çizmek ---
    if (hedef_bulundu) {
        ofstream hedef_file("hedef_nokta.dat");
        hedef_file << en_yakin_hedef.p.x << " " << en_yakin_hedef.p.y << "\n";
        hedef_file.close();

        ofstream mesafe_file("mesafe_cizgisi.dat");
        mesafe_file << "0 0\n"; // robotun başlangıç noktası
        mesafe_file << en_yakin_hedef.p.x << " " << en_yakin_hedef.p.y << "\n"; // Bitiş: Hedef
        mesafe_file.close();

        double label_x = en_yakin_hedef.p.x + 0.1;
        double label_y = en_yakin_hedef.p.y + 0.1;

        stringstream label_ss;
        label_ss.precision(2);
        label_ss << "'Kesisim: (d" << en_yakin_hedef.dogru_index1 + 1 << " & d" << en_yakin_hedef.dogru_index2 + 1 << ") "
            << "Aci: " << (int)en_yakin_hedef.aci << " derece "
            << "Mesafe: " << fixed << en_yakin_hedef.robota_mesafe << "m'";
        script_file << "\n# --- Hedef Etiketleri ---\n";
        script_file << "set label 1 " << label_ss.str() << " at " << label_x << "," << label_y << " font ',10' textcolor 'black' front\n";
    }
    
    // --- Doğruların Orta Noktalarına Etiket(LABEL) Ekleme ---
    script_file << "\n# --- Dogru Etiketleri ---\n";
    for(size_t i = 0; i < tespitEdilenDogrular.size(); ++i) {
        Dogru d = tespitEdilenDogrular[i];
        if (d.noktalar.empty()) continue;

        double mid_x, mid_y;
        
        if (d.dikey_mi) {
            double min_y = d.noktalar[0].y;
            double max_y = d.noktalar[0].y;
            for (const Nokta& p : d.noktalar) {
                if (p.y < min_y) min_y = p.y;
                if (p.y > max_y) max_y = p.y;
            }
            mid_x = d.x_kesme_noktasi;
            mid_y = (min_y + max_y) / 2.0;
        } else {
            double min_x = d.noktalar[0].x;
            double max_x = d.noktalar[0].x;
            for (const Nokta& p : d.noktalar) {
                if (p.x < min_x) min_x = p.x;
                if (p.x > max_x) max_x = p.x;
            }
            double y1 = d.m * min_x + d.c;
            double y2 = d.m * max_x + d.c;
            mid_x = (min_x + max_x) / 2.0;
            mid_y = (y1 + y2) / 2.0;
        }
        
        script_file << "set label " << i + 2 << " 'd" << i + 1 << "' at " << mid_x << "," << mid_y << " font ',7' textcolor 'black' offset 0,0.2 front\n";
    }

    script_file << "\n# --- Cizim Komutlari ---\n";
    script_file << "plot \\\n";
    script_file << "   'points.dat' with points pt 7 ps 0.5 lc 'blue' title 'Gecerli Noktalar', \\\n";
    script_file << "   'robot.dat' with points pt 7 ps 1.5 lc 'red' title 'Robot Konumu'";
    
    // Tespit Edilen Doğruları Çiz
    for (size_t i = 0; i < tespitEdilenDogrular.size(); ++i) {
        Dogru d = tespitEdilenDogrular[i];
        if (d.noktalar.empty()) continue; // Noktası olmayan doğruyu atla

        script_file << ", \\\n";

        stringstream title_ss;
        title_ss << "Dogru " << i + 1 << " (" << d.noktalar.size() << " nokta)";
        
        bool is_hedef_dogrusu = false;
        if (hedef_bulundu && (i == en_yakin_hedef.dogru_index1 || i == en_yakin_hedef.dogru_index2)) {
            is_hedef_dogrusu = true;
        }
        
        string line_filename = "dogru_" + to_string(i) + ".dat";
        ofstream line_file(line_filename);

        if (d.dikey_mi) {
            double min_y = d.noktalar[0].y, max_y = d.noktalar[0].y;
            for (const Nokta& p : d.noktalar) {
                if (p.y < min_y) min_y = p.y;
                if (p.y > max_y) max_y = p.y;
            }
            Nokta p1 = {d.x_kesme_noktasi, min_y};
            Nokta p2 = {d.x_kesme_noktasi, max_y};

            if (is_hedef_dogrusu) {
                // Hedef doğrusunu kesişime kadar uzat
                double dist1 = mesafeHesapla(en_yakin_hedef.p, p1);
                double dist2 = mesafeHesapla(en_yakin_hedef.p, p2);
                if (dist1 > dist2) { // p1 daha uzak, p1'den kesişime çiz
                    line_file << p1.x << " " << p1.y << "\n";
                    line_file << en_yakin_hedef.p.x << " " << en_yakin_hedef.p.y << "\n";
                } else { // p2 daha uzak, p2'den kesişime çiz
                    line_file << p2.x << " " << p2.y << "\n";
                    line_file << en_yakin_hedef.p.x << " " << en_yakin_hedef.p.y << "\n";
                }
            } else { // Normal doğruyu kendi segmentinde çiz
                line_file << p1.x << " " << p1.y << "\n";
                line_file << p2.x << " " << p2.y << "\n";
            }
        } else { // Dikey olmayan doğru
            double min_x = d.noktalar[0].x, max_x = d.noktalar[0].x;
            for (const Nokta& p : d.noktalar) {
                if (p.x < min_x) min_x = p.x;
                if (p.x > max_x) max_x = p.x;
            }
            Nokta p1 = {min_x, d.m * min_x + d.c};
            Nokta p2 = {max_x, d.m * max_x + d.c};
            
            if (is_hedef_dogrusu) {
                 // Hedef doğrusunu kesişime kadar uzat
                double dist1 = mesafeHesapla(en_yakin_hedef.p, p1);
                double dist2 = mesafeHesapla(en_yakin_hedef.p, p2);
                if (dist1 > dist2) { // p1 daha uzak
                    line_file << p1.x << " " << p1.y << "\n";
                    line_file << en_yakin_hedef.p.x << " " << en_yakin_hedef.p.y << "\n";
                } else { // p2 daha uzak
                    line_file << p2.x << " " << p2.y << "\n";
                    line_file << en_yakin_hedef.p.x << " " << en_yakin_hedef.p.y << "\n";
                }
            } else { // Normal doğruyu kendi segmentinde çiz
                line_file << p1.x << " " << p1.y << "\n";
                line_file << p2.x << " " << p2.y << "\n";
            }
        }
        line_file.close();
        script_file << "   '" << line_filename << "' with lines lw 2 lc 'green' title '" << title_ss.str() << "'";
    }

    // Hedefi ve Mesafeyi Çiz
    if (hedef_bulundu) {
        script_file << ", \\\n";
        script_file << "   'hedef_nokta.dat' with points pt 7 ps 1.5 lc 'yellow' title '60+ Kesisim'";
        script_file << ", \\\n";
        script_file << "   'mesafe_cizgisi.dat' with lines lt 2 lw 2 dashtype 4 lc 'red'  title 'Mesafe Cizgisi'";
    }

    script_file << "\n"; // Plot komutunu sonlandırmak için
    script_file.close();
    cout << "Gnuplot script'i (plot_script.gp) GUNCELLENDI." << endl;

    cout << "Grafik ciziliyor..." << endl;
    system("gnuplot -p plot_script.gp");

    return 0;
}