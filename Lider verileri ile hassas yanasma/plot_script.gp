# --- Genel Ayarlar ---
set terminal wxt enhanced
set title 'Lidar Verisi (Hedef Tespiti ile)'
set xlabel 'X Ekseni (m)'
set ylabel 'Y Ekseni (m)'
set grid
set size square
set xrange [-3:3]
set yrange [-3:3]
set key right top outside
set key box linetype 1 linewidth 1.5
set key title 'Gosterim'
set key spacing 1.2
set key samplen 2

# --- Hedef Etiketleri ---
set label 1 'Kesisim: (d1 & d4) Aci: 73 derece Mesafe: 1.52m' at 1.36925,0.942981 font ',10' textcolor 'black' front

# --- Dogru Etiketleri ---
set label 2 'd1' at 0.767661,1.20458 font ',7' textcolor 'black' offset 0,0.2 front
set label 3 'd2' at 0.447891,1.23676 font ',7' textcolor 'black' offset 0,0.2 front
set label 4 'd3' at 1.24345,0.769079 font ',7' textcolor 'black' offset 0,0.2 front
set label 5 'd4' at 1.10781,0.372556 font ',7' textcolor 'black' offset 0,0.2 front
set label 6 'd5' at -0.101035,1.62994 font ',7' textcolor 'black' offset 0,0.2 front

# --- Cizim Komutlari ---
plot \
   'points.dat' with points pt 7 ps 0.5 lc 'blue' title 'Gecerli Noktalar', \
   'robot.dat' with points pt 7 ps 1.5 lc 'red' title 'Robot Konumu', \
   'dogru_0.dat' with lines lw 2 lc 'green' title 'Dogru 1 (22 nokta)', \
   'dogru_1.dat' with lines lw 2 lc 'green' title 'Dogru 2 (15 nokta)', \
   'dogru_2.dat' with lines lw 2 lc 'green' title 'Dogru 3 (11 nokta)', \
   'dogru_3.dat' with lines lw 2 lc 'green' title 'Dogru 4 (8 nokta)', \
   'dogru_4.dat' with lines lw 2 lc 'green' title 'Dogru 5 (9 nokta)', \
   'hedef_nokta.dat' with points pt 7 ps 1.5 lc 'yellow' title '60+ Kesisim', \
   'mesafe_cizgisi.dat' with lines lt 2 lw 2 dashtype 4 lc 'red'  title 'Mesafe Cizgisi'
