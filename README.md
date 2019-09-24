# FIBOT - robot 4-kołowy
W skład robota wchodzi:

-płyta sterująca
-Raspberry Pi 3 + kamera osadzona na dwóch serwomechanizmach modelarskich
-4 silniki DC
-4 czujniki ultradźwiękowe
-oraz zasilanie z baterii LI-ion z BMS.

Płyta sterująca została wyposażona w mikrokontroler MSP430FR5739, na który w języku C napisano oprogramowanie za pomocą Code Composer Studio.
Płyta sterująca odpowiada za sterowanie czterema silnikami DC z wykorzystaniem PID, odczyt enkoderów umieszczonych na silnikach, pomiar napięcia zasilania, pomiar odległości za pomocą czujników ultradźwiękowych, pomiar prądu dla każdego silnika, obsługa diody i buzzera, komunikacja z Raspberry Pi 3 za pomocą UART.

Na Raspberry Pi 3 zainstalowano Ubuntu Mate oraz Robot Operating System.
Raspberry Pi 3 odbiera informacje z stacji sterującej ( Laptop HP z Ubuntu i ROS + joystick) odnośnie toru poruszania się robota i przesyłą po UART informacje do płyty sterującej. Raspi przesyła obraz do stacji sterującej.


![20190222_201828_1](https://user-images.githubusercontent.com/33237349/65499621-31abb580-debe-11e9-94ba-2c148f954f9d.jpg)
![20190222_201916](https://user-images.githubusercontent.com/33237349/65497768-0c697800-debb-11e9-9fe7-621a2f85fbf2.jpg)
