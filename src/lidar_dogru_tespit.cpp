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
        if (abs(1.0 + d1.m * d2.m) < 0.0001) { return 90.0; } // Dik kesişim (payda ~0)
        double tan_theta = abs((d1.m - d2.m) / (1.0 + d1.m * d2.m));
        return atan(tan_theta) * 180.0 / PI;
    }
    return 0.0; // Beklenmedik durum
}

Nokta ikiDogruKesisim(Dogru d1, Dogru d2, bool& bulundu) {
    Nokta kesisim_p = { 0.0, 0.0 }; // Başlangıç değeri (MSVC C4700 uyarısını önler)
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

    // Paralel (payda ~0) veya geçersiz eğim
    if (isnan(d1.m) || isnan(d2.m) || abs(d1.m - d2.m) < 0.0001) {
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


// --- VERİ OKUMA ---

struct TaramaVerisi {
    double anglemin = 0.0, anglemax = 0.0, angleincrement = 0.0;
    double rangemin = 0.0, rangemax = 0.0;
    vector<double> ranges;
};

// Komut satırına göre modu seçer, gerekirse dosyayı indirir ve kullanılacak dosyanın adını döndürür.
// Kullanım:
//   lidar_dogru_tespit                -> örnek veri (ornek-veri/scan_data_NaN.toml)
//   lidar_dogru_tespit dosya.toml     -> verilen dosya
//   lidar_dogru_tespit --sunum        -> sunucudan rastgele dosya indir
string veriDosyasiniBelirle(int argc, char* argv[]) {
    const string varsayilan_dosya = "ornek-veri/scan_data_NaN.toml";
    bool test_modu = true;
    string yerel_test_dosyasi = varsayilan_dosya;

    if (argc > 1) {
        string arguman = argv[1];
        if (arguman == "--sunum") {
            test_modu = false;
        }
        else {
            yerel_test_dosyasi = arguman;
        }
    }

    if (test_modu) {
        srand(1); // Test modunda her çalışmada aynı sonuç
        cout << "--- TEST MODU AKTIF ---" << endl;
        cout << "Yerel test dosyasi kullaniliyor: " << yerel_test_dosyasi << endl;
        return yerel_test_dosyasi;
    }

    // --- INTERNETTEN INDIRME ---
    srand(time(NULL)); // sunum modu için her seferinde farklı rastgelelik
    int dosya_numarasi = (rand() % 5) + 1;
    string base_url = "http://abilgisayar.kocaeli.edu.tr/lidar";
    string file_extension = ".toml";
    string url = base_url + to_string(dosya_numarasi) + file_extension;
    string indirilen_dosya_adi = "indirilen_scan.toml";
    string download_command = "curl -f -o " + indirilen_dosya_adi + " " + url; // -f: HTTP hatasında (404/403) başarısız say

    cout << "--- SUNUM MODU AKTIF ---" << endl;
    cout << "Rastgele secilen dosya: lidar" << dosya_numarasi << ".toml" << endl;
    cout << "Dosya indiriliyor: " << url << endl;
    int result = system(download_command.c_str());
    if (result != 0) {
        cout << "Dosya indirme basarisiz! 'curl' kurulu ve internet baglantisi oldugundan emin olun." << endl;
        cout << "Indirme basarisiz, yerel test dosyasi kullanilacak..." << endl;
        return varsayilan_dosya; // Hata durumunda örnek veriyle devam et
    }
    cout << "Dosya basariyla indirildi: " << indirilen_dosya_adi << endl;
    return indirilen_dosya_adi;
}

// TOML dosyasındaki tarama parametrelerini ve 'ranges' dizisini harici kütüphane kullanmadan okur.
bool tomlDosyasiniOku(const string& dosya_adi, TaramaVerisi& veri) {
    ifstream dosya(dosya_adi);
    if (!dosya.is_open()) {
        cout << "Kullanilacak dosya '" << dosya_adi << "' acilamadi!" << endl;
        return false;
    }
    cout << "Dosya acildi." << endl;

    string kelime;
    while (getline(dosya, kelime)) {
        if (kelime.find("angle_min") != string::npos) {
            veri.anglemin = stod(kelime.substr(kelime.find("=") + 1));
        }
        else if (kelime.find("angle_max") != string::npos) {
            veri.anglemax = stod(kelime.substr(kelime.find("=") + 1));
        }
        else if (kelime.find("angle_increment") != string::npos) {
            veri.angleincrement = stod(kelime.substr(kelime.find("=") + 1));
        }
        else if (kelime.find("range_min") != string::npos) {
            veri.rangemin = stod(kelime.substr(kelime.find("=") + 1));
        }
        else if (kelime.find("range_max") != string::npos) {
            veri.rangemax = stod(kelime.substr(kelime.find("=") + 1));
        }
        else if (kelime.find("ranges") != string::npos) {
            // Dizi birden fazla satıra yayılabilir: ']' görülene kadar oku
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
                        veri.ranges.push_back(NAN);
                    }
                    else {
                        try {
                            veri.ranges.push_back(stod(temp_val));
                        }
                        catch (const std::invalid_argument&) {} // Sayı olmayan değerleri atla
                    }
                }
                if (in_array) {
                    getline(dosya, current_line);
                }
            }
        }
    }
    return true;
}

// --- NOKTALARI FİLTRELEME VE DÖNÜŞÜM ---

// Geçersiz ölçümleri eler, kutupsal (r, θ) değerleri kartezyen (x, y) noktalara çevirir.
// Tüm geçerli noktalar points.dat'a yazılır; RANSAC için sadece 50cm'den uzak olanlar döndürülür.
vector<Nokta> noktalariFiltreleVeDonustur(const TaramaVerisi& veri) {
    ofstream robot_file("robot.dat");
    robot_file << "0 0\n";
    robot_file.close();

    ofstream points_file("points.dat");
    vector<Nokta> gecerli_noktalar; // RANSAC için
    int nantutucu = 0;
    int toplam_gecerli_nokta_sayaci = 0; // Görsel için

    const double RANSAC_MIN_MESAFESI = 0.5; // 50cm

    for (size_t i = 0; i < veri.ranges.size(); ++i) {
        double r = veri.ranges[i];
        if (isnan(r) || r < veri.rangemin || r > veri.rangemax) {
            nantutucu++;
            continue;
        }
        toplam_gecerli_nokta_sayaci++;
        double angle = veri.anglemin + i * veri.angleincrement;
        double x = r * cos(angle);
        double y = r * sin(angle);
        points_file << x << " " << y << "\n";
        if (r > RANSAC_MIN_MESAFESI) {
            gecerli_noktalar.push_back({ x, y });
        }
    }
    points_file.close();
    cout << "Filtrelenen (gecersiz/NaN) nokta sayisi: " << nantutucu << endl;
    cout << "Gecerli nokta sayisi (Gorsel icin): " << toplam_gecerli_nokta_sayaci << endl;
    cout << "Gecerli nokta sayisi (RANSAC icin): " << gecerli_noktalar.size() << " (50cm filtresi uygulandi)" << endl;
    return gecerli_noktalar;
}

// --- DOĞRU TESPİTİ: RANSAC ---

const double UZAKLIK_TOLERANSI = 0.02;     // 2cm (noktanın doğruya uzaklığı)
const size_t MIN_DESTEKCI_SAYISI = 8;
const int ITERASYON_SAYISI = 3000;
const double MAX_NOKTA_ARASI_MESAFE = 1.0; // Bir doğru üzerindeki ardışık iki nokta arası max mesafe (1m)
const size_t MAX_DOGRU_SAYISI = 10;

// Rastgele iki noktadan geçen aday doğruyu oluşturur.
Dogru ikiNoktadanDogru(Nokta p1, Nokta p2) {
    Dogru d;
    if (abs(p1.x - p2.x) < 0.0001) {
        d.dikey_mi = true;
        d.x_kesme_noktasi = p1.x;
        d.m = NAN;
        d.c = NAN;
    }
    else {
        d.dikey_mi = false;
        d.m = (p2.y - p1.y) / (p2.x - p1.x);
        d.c = p1.y - d.m * p1.x;
        d.x_kesme_noktasi = NAN;
    }
    return d;
}

// En çok noktanın desteklediği aday doğrunun destekçi noktalarını döndürür.
vector<Nokta> enIyiDestekcileriBul(const vector<Nokta>& noktalar) {
    vector<Nokta> en_iyi_destekciler;
    for (int i = 0; i < ITERASYON_SAYISI; ++i) {
        int index1 = rand() % (int)noktalar.size();
        int index2 = rand() % (int)noktalar.size();
        if (index1 == index2) continue;
        Dogru aday_dogru = ikiNoktadanDogru(noktalar[index1], noktalar[index2]);

        vector<Nokta> guncel_destekciler;
        for (const Nokta& p : noktalar) {
            if (noktadanDogruyaUzaklik(p, aday_dogru) < UZAKLIK_TOLERANSI) {
                guncel_destekciler.push_back(p);
            }
        }
        if (guncel_destekciler.size() > en_iyi_destekciler.size()) {
            en_iyi_destekciler = guncel_destekciler;
        }
    }
    return en_iyi_destekciler;
}

// Doğru üzerinde sıralanmış noktaları, aralarında büyük boşluk olan yerlerden parçalara ayırır.
// Böylece aynı doğrultudaki ama birbirinden uzak iki duvar tek doğru sayılmaz.
vector<vector<Nokta>> surekliParcalaraAyir(vector<Nokta> destekciler) {
    // Destekçi noktaları en küçük kareler ile bulunan model üzerinde sırala
    Dogru model = dogruyaEnIyiUyanCizgiyiBul(destekciler);
    sort(destekciler.begin(), destekciler.end(), [&](const Nokta& a, const Nokta& b) {
        if (model.dikey_mi) {
            return a.y < b.y; // Dikey doğru ise y eksenine göre sırala
        }
        return a.x < b.x; // Normal doğruları x eksenine göre sırala
    });

    vector<vector<Nokta>> parcalar;
    if (destekciler.empty()) return parcalar;
    parcalar.push_back({ destekciler[0] });
    for (size_t k = 1; k < destekciler.size(); ++k) {
        if (mesafeHesapla(destekciler[k], destekciler[k - 1]) <= MAX_NOKTA_ARASI_MESAFE) {
            parcalar.back().push_back(destekciler[k]);
        }
        else {
            parcalar.push_back({ destekciler[k] });
        }
    }
    return parcalar;
}

// Kullanılan noktaları listeden çıkarır (bir sonraki RANSAC turu için).
vector<Nokta> kullanilanlariCikar(const vector<Nokta>& noktalar, const vector<Nokta>& kullanilanlar) {
    vector<Nokta> kalan;
    for (const Nokta& p_kalan : noktalar) {
        bool bulundu = false;
        for (const Nokta& p_kullanilan : kullanilanlar) {
            if (abs(p_kalan.x - p_kullanilan.x) < 0.0001 && abs(p_kalan.y - p_kullanilan.y) < 0.0001) {
                bulundu = true;
                break;
            }
        }
        if (!bulundu) {
            kalan.push_back(p_kalan);
        }
    }
    return kalan;
}

// RANSAC + en küçük kareler ile nokta bulutundaki doğruları bulur.
vector<Dogru> ransacIleDogrulariBul(const vector<Nokta>& gecerli_noktalar) {
    vector<Dogru> tespitEdilenDogrular;
    vector<Nokta> kalan_noktalar = gecerli_noktalar;

    while (kalan_noktalar.size() >= MIN_DESTEKCI_SAYISI && tespitEdilenDogrular.size() < MAX_DOGRU_SAYISI) {
        vector<Nokta> en_iyi_destekciler = enIyiDestekcileriBul(kalan_noktalar);
        if (en_iyi_destekciler.size() < MIN_DESTEKCI_SAYISI) {
            break; // Yeterli destekçi bulunamadıysa, daha fazla doğru aramanın anlamı yok.
        }

        // Yeterli sayıda noktaya sahip parçaları geçerli doğru olarak kabul et
        vector<Nokta> tum_kullanilan_noktalar;
        for (const auto& parca : surekliParcalaraAyir(en_iyi_destekciler)) {
            if (parca.size() >= MIN_DESTEKCI_SAYISI) {
                // Her parça için sadece kendi noktalarıyla son kez en uygun doğruyu hesapla
                tespitEdilenDogrular.push_back(dogruyaEnIyiUyanCizgiyiBul(parca));
                tum_kullanilan_noktalar.insert(tum_kullanilan_noktalar.end(), parca.begin(), parca.end());
            }
        }

        if (tum_kullanilan_noktalar.empty()) {
            break;
        }
        kalan_noktalar = kullanilanlariCikar(kalan_noktalar, tum_kullanilan_noktalar);
    }

    cout << "------------------------" << endl;
    cout << "RANSAC TAMAMLANDI." << endl;
    cout << "Tespit Edilen Dogru Sayisi: " << tespitEdilenDogrular.size() << endl;
    for (size_t i = 0; i < tespitEdilenDogrular.size(); ++i) {
        cout << "  Dogru " << i + 1 << ": " << tespitEdilenDogrular[i].noktalar.size() << " adet noktaya sahip." << endl;
    }
    cout << "------------------------" << endl;
    return tespitEdilenDogrular;
}

// --- GEOMETRİK ANALİZ ---

const double MIN_KESISIM_ACISI = 60.0;

// Her doğru çiftinin kesişimini bulur, 60 derece ve üzeri olanları döndürür.
vector<KesisimNoktasi> onemliKesisimleriBul(const vector<Dogru>& dogrular) {
    vector<KesisimNoktasi> onemliKesisimler;
    const Nokta robot_konumu = { 0.0, 0.0 };
    cout << "GEOMETRIK ANALIZ BASLADI..." << endl;

    for (size_t i = 0; i < dogrular.size(); ++i) {
        for (size_t j = i + 1; j < dogrular.size(); ++j) {
            bool kesisim_var_mi = false;
            Nokta kesisim_p = ikiDogruKesisim(dogrular[i], dogrular[j], kesisim_var_mi);
            if (!kesisim_var_mi) continue;

            double aci = ikiDogruAci(dogrular[i], dogrular[j]);
            cout << "   - Dogru " << i + 1 << " ve Dogru " << j + 1 << " kesisiyor. Aci: " << aci << " derece." << endl;

            if (aci >= MIN_KESISIM_ACISI) {
                KesisimNoktasi yeni_onemli_nokta;
                yeni_onemli_nokta.p = kesisim_p;
                yeni_onemli_nokta.aci = aci;
                yeni_onemli_nokta.robota_mesafe = mesafeHesapla(robot_konumu, kesisim_p);
                yeni_onemli_nokta.dogru_index1 = i;
                yeni_onemli_nokta.dogru_index2 = j;
                onemliKesisimler.push_back(yeni_onemli_nokta);
            }
        }
    }

    cout << "------------------------" << endl;
    cout << "ANALIZ TAMAMLANDI." << endl;
    cout << "Toplam 60+ derece Kesisim Sayisi: " << onemliKesisimler.size() << endl;
    return onemliKesisimler;
}

// Kesişimler içinden robota en yakın olanı seçer. Hiç kesişim yoksa false döner.
bool enYakinHedefiSec(const vector<KesisimNoktasi>& kesisimler, KesisimNoktasi& en_yakin_hedef) {
    bool hedef_bulundu = !kesisimler.empty();
    if (hedef_bulundu) {
        en_yakin_hedef = kesisimler[0];
        for (const KesisimNoktasi& k : kesisimler) {
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
    return hedef_bulundu;
}

// --- GNUPLOT SCRIPT OLUŞTURMA ---

// Doğrunun kendi noktalarının kapsadığı parçanın iki uç noktasını bulur.
void dogruUcNoktalariniBul(const Dogru& d, Nokta& p1, Nokta& p2) {
    if (d.dikey_mi) {
        double min_y = d.noktalar[0].y, max_y = d.noktalar[0].y;
        for (const Nokta& p : d.noktalar) {
            if (p.y < min_y) min_y = p.y;
            if (p.y > max_y) max_y = p.y;
        }
        p1 = { d.x_kesme_noktasi, min_y };
        p2 = { d.x_kesme_noktasi, max_y };
    }
    else {
        double min_x = d.noktalar[0].x, max_x = d.noktalar[0].x;
        for (const Nokta& p : d.noktalar) {
            if (p.x < min_x) min_x = p.x;
            if (p.x > max_x) max_x = p.x;
        }
        p1 = { min_x, d.m * min_x + d.c };
        p2 = { max_x, d.m * max_x + d.c };
    }
}

// Doğruları, hedefi ve mesafe çizgisini .dat dosyalarına yazar; bunları çizen plot_script.gp'yi oluşturur.
bool gnuplotBetigiYaz(const vector<Dogru>& dogrular, bool hedef_bulundu, const KesisimNoktasi& en_yakin_hedef) {
    ofstream script_file("plot_script.gp");
    if (!script_file.is_open()) {
        std::cerr << "plot_script.gp dosyasi olusturulamadi!" << std::endl;
        return false;
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

        stringstream label_ss;
        label_ss.precision(2);
        label_ss << "'Kesisim: (d" << en_yakin_hedef.dogru_index1 + 1 << " & d" << en_yakin_hedef.dogru_index2 + 1 << ") "
            << "Aci: " << (int)en_yakin_hedef.aci << " derece "
            << "Mesafe: " << fixed << en_yakin_hedef.robota_mesafe << "m'";
        script_file << "\n# --- Hedef Etiketleri ---\n";
        script_file << "set label 1 " << label_ss.str() << " at " << en_yakin_hedef.p.x + 0.1 << "," << en_yakin_hedef.p.y + 0.1
            << " font ',10' textcolor 'black' front\n";
    }

    // --- Doğruların Orta Noktalarına Etiket(LABEL) Ekleme ---
    script_file << "\n# --- Dogru Etiketleri ---\n";
    for (size_t i = 0; i < dogrular.size(); ++i) {
        if (dogrular[i].noktalar.empty()) continue;
        Nokta p1, p2;
        dogruUcNoktalariniBul(dogrular[i], p1, p2);
        double mid_x = (p1.x + p2.x) / 2.0;
        double mid_y = (p1.y + p2.y) / 2.0;
        script_file << "set label " << i + 2 << " 'd" << i + 1 << "' at " << mid_x << "," << mid_y << " font ',7' textcolor 'black' offset 0,0.2 front\n";
    }

    script_file << "\n# --- Cizim Komutlari ---\n";
    script_file << "plot \\\n";
    script_file << "   'points.dat' with points pt 7 ps 0.5 lc 'blue' title 'Gecerli Noktalar', \\\n";
    script_file << "   'robot.dat' with points pt 7 ps 1.5 lc 'red' title 'Robot Konumu'";

    // Tespit Edilen Doğruları Çiz
    for (size_t i = 0; i < dogrular.size(); ++i) {
        const Dogru& d = dogrular[i];
        if (d.noktalar.empty()) continue; // Noktası olmayan doğruyu atla

        Nokta p1, p2;
        dogruUcNoktalariniBul(d, p1, p2);

        string line_filename = "dogru_" + to_string(i) + ".dat";
        ofstream line_file(line_filename);
        bool is_hedef_dogrusu = hedef_bulundu && (i == en_yakin_hedef.dogru_index1 || i == en_yakin_hedef.dogru_index2);
        if (is_hedef_dogrusu) {
            // Hedef doğrusunu, uzak ucundan kesişim noktasına kadar uzat
            Nokta uzak_uc = (mesafeHesapla(en_yakin_hedef.p, p1) > mesafeHesapla(en_yakin_hedef.p, p2)) ? p1 : p2;
            line_file << uzak_uc.x << " " << uzak_uc.y << "\n";
            line_file << en_yakin_hedef.p.x << " " << en_yakin_hedef.p.y << "\n";
        }
        else { // Normal doğruyu kendi segmentinde çiz
            line_file << p1.x << " " << p1.y << "\n";
            line_file << p2.x << " " << p2.y << "\n";
        }
        line_file.close();

        stringstream title_ss;
        title_ss << "Dogru " << i + 1 << " (" << d.noktalar.size() << " nokta)";
        script_file << ", \\\n";
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
    return true;
}


// --- ANA FONKSİYON ---

int main(int argc, char* argv[])
{
    string kullanilacak_dosya_adi = veriDosyasiniBelirle(argc, argv);
    cout << "------------------------" << endl;

    TaramaVerisi veri;
    if (!tomlDosyasiniOku(kullanilacak_dosya_adi, veri)) {
        return 1;
    }

    cout << "------------------------" << endl;
    cout << "angle_min: " << veri.anglemin << endl;
    cout << "angle_max: " << veri.anglemax << endl;
    cout << "range_min: " << veri.rangemin << endl;
    cout << "range_max: " << veri.rangemax << endl;
    cout << "Okunan 'ranges' eleman sayisi: " << veri.ranges.size() << endl;

    vector<Nokta> gecerli_noktalar = noktalariFiltreleVeDonustur(veri);
    vector<Dogru> dogrular = ransacIleDogrulariBul(gecerli_noktalar);
    vector<KesisimNoktasi> kesisimler = onemliKesisimleriBul(dogrular);

    KesisimNoktasi en_yakin_hedef;
    bool hedef_bulundu = enYakinHedefiSec(kesisimler, en_yakin_hedef);

    if (!gnuplotBetigiYaz(dogrular, hedef_bulundu, en_yakin_hedef)) {
        return 1;
    }

    cout << "Grafik ciziliyor..." << endl;
    if (system("gnuplot -p plot_script.gp") != 0) {
        cout << "Gnuplot calistirilamadi. Kurulu oldugundan emin olun; grafigi daha sonra 'gnuplot -p plot_script.gp' ile acabilirsiniz." << endl;
    }

    return 0;
}
