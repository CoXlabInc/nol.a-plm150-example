#include <cox.h>

Timer timerHello;

#define LD2    PA5
#define BUTTON PC13

void setup() {
  Serial.begin(115200);
  Serial.printf("\n*** [ST Nucleo-L073RZ] Basic Functions ***\n");
  Serial.listen();
  Serial.onReceive([](SerialPort &p) {
    while (p.available() > 0) {
      char c = p.read();
      p.write(c);
    }
  });

  pinMode(LD2, OUTPUT);

  timerHello.onFired(
    [](void *) {
      printf("Hello World!\n");
      digitalToggle(LD2);
    },
    NULL
  );
  timerHello.startPeriodic(1000);

  attachInterrupt(
    BUTTON,
    []() {
      printf("Button pressed!\n");
      if (Serial.isListening()) {
        Serial.stopListening();
        printf("* Stop Serial listening.\n");
      } else {
        printf("* Listen on Serial.\n");
        Serial.listen();
      }
    },
    FALLING
  );
}
