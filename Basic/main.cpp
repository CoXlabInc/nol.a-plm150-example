#include <cox.h>

Timer timerHello;

void setup() {
  Serial.begin(115200);
  Serial.printf("\n*** [PLM150] Basic Functions ***\n");
  Serial.listen();
  Serial.onReceive([](SerialPort &p) {
    while (p.available() > 0) {
      char c = p.read();
      p.write(c);
    }
  });

  timerHello.onFired(
    [](void *) {
      printf("Hello World!\n");
    },
    NULL
  );
  timerHello.startPeriodic(1000);
}
