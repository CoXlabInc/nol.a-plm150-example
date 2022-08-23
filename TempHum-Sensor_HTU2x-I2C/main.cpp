#include <cox.h>
#include <dev/HTU2xD.hpp>

Timer timerHello;

HTU2xD htu(Wire3);

void setup() {
  Serial.begin(115200);
  Serial.printf("\n*** [PLM150] Temperature & Humidity Sensor ***\n");
  Serial.listen();
  Serial.onReceive([](SerialPort &p) {
    while (p.available() > 0) {
      char c = p.read();
      p.write(c);
    }
  });

  htu.begin();

  timerHello.onFired(
    [](void *) {
      printf("%f celcius degree, %f %%\n", htu.readTemperature(), htu.readHumidity());
    }
  );
  timerHello.startPeriodic(1000);
}
