# pionier_key
Przykładowy skrypt do sterowania robotem / Bez RosAria.

Trzeba znaleźć plik przez który ubuntu przepuszcza informacje z klawiatury ->
$ cat /proc/bus/input/devices

Znaleźć wpis z klawiaturą, coś w ten deseń:
I: Bus=0011 Vendor=0001 Product=0001 Version=ab41
N: Name="AT Translated Set 2 keyboard"
P: Phys=isa0060/serio0/input0
S: Sysfs=/devices/platform/i8042/serio0/input/inputX   # ten numer trzeba użyć odpalając dockera
U: Uniq=
 
W wywołaniu dockera (docker run [bla bla]) przed nazwą obrazu należy dopisać '--devices=/dev/input/eventX' X - zastąpić numerem urządzenia wejściowego (np. '--devices=/dev/input/event1')

W pliku pionier_key/inc/keyboard.h należy zmienić ścieżkę do klawiatury na tę samą, którą przekazujemy do dockera - '/dev/input/eventX'

W razie braku SDL/SDL.h doinstalować bibliotekę libsdl:
sudo apt-get install libsdl1.2-dev


// Instalacja paczki
cd ~/catkin_ws/src
git clone https://github.com/ProjektPrzejsciowy/pionier_key.git
cd ..
catkin_make
// ewentualnie komenda budujaca jedynia ta jedna paczke
catkin_make --pkg pionier_key

rosrun pionier_key pionier_unstopableCircle
//w programie przyciski s/q startują/wyłączają sterowanie (wielkość liter się liczy :P) 
