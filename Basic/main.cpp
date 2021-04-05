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
      struct timeval tNow;
      gettimeofday(&tNow, nullptr);
      printf("[%lu.%06lu] Hello World!\n", (uint32_t) tNow.tv_sec, tNow.tv_usec);
    },
    nullptr
  );
  timerHello.startPeriodic(1000);
}
