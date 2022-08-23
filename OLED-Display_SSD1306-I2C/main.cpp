#include <cox.h>
#include <dev/Adafruit_SSD1306.hpp>

Timer timerHello;

Adafruit_SSD1306 display(128, 64, &Wire3);

void setup() {
  Serial.begin(115200);
  Serial.printf("\n*** [PLM150] OLED Display (SSD1306 I2C) ***\n");
  Serial.listen();
  Serial.onReceive([](SerialPort &p) {
    while (p.available() > 0) {
      char c = p.read();
      p.write(c);
    }
  });

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("*** [PLM150] ***");
  display.display();

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
