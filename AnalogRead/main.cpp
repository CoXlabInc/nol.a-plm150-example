#include <cox.h>
#include <dev/HTU2xD.hpp>

Timer timerHello;

void setup() {
  Serial.begin(115200);
  Serial.printf("\n*** [PLM150] Analog Read ***\n");
  Serial.listen();
  Serial.onReceive([](SerialPort &p) {
    while (p.available() > 0) {
      char c = p.read();
      p.write(c);
    }
  });

  pinMode(31, INPUT);

  timerHello.onFired(
    [](void *) {
      analogReadResolution(12);
      printf("%ld\n", analogRead(31));
    }
  );
  timerHello.startPeriodic(1000);
}
