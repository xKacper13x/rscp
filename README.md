### Status Prac: ROS Bridge & Transceiver


**Zrobione:**
* Kodowanie/dekodowanie COBS, parsowanie `RequestEnvelope` i `ResponseEnvelope`.
* Rzucanie wyjątkami przy złych typach, hermetyzacja w `Dataclassy`.
* Wysyłanie i odbieranie danych przez port szeregowy.
* Zaimplementowane wysyłanie wiadomości: `Acknowledge`, `TaskFinished`, `GPSCoordinate`, `RoverStatus`, `BatteryState`, `distance` (*Uwaga: na razie brak implementacji dla ogólnego stringa `message`*).


**TODO (Do uzgodnienia i implementacji):**
Uzupełnienie prawdziwych nazw topików z ROS-a, zrobienie subskrybentów oraz odpalenie Timera 1Hz dla RoverStatus. Rozróżnianie innych poleceń niż navigate_to_gps


**Sposób testowania lokalnego:**
1. Zainstaluj zależności: `pip install -r requirements.txt`
2. Uruchom wirtualne porty: `socat -d -d pty,raw,echo=0 pty,raw,echo=0`
 ( trzeba je wpisać w main.py i rscp_transceiver.py)
3. Zaktualizuj wygenerowane ścieżki portów (np. `/dev/pts/X`) w plikach `main.py` oraz `rscp_transceiver.py`.
4. [Terminal 1] Uruchom główny węzeł: `python3 rscp_ros_bridge.py`
5. [Terminal 2] Uruchom mock stacji bazowej: `python3 main.py`
6. Natychmiastowo w pierwszym terminalu powinna pojawić się informacja o otrzymaniu polcenia 'navigate_to_gps',
    a w drugim powiadomienie o otrzymaniu ACK. Po 3 sekundach w drugim terminalu pojawia się wynik zadania i
    informacja o jego zakończeniu.