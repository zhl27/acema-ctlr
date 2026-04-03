#include <HX711_ADC.h>
#include <SerialPrint.h>

const int boton_emergencia = 3;
const int boton_ignicion = 4;
const int boton_onoff = 5;
const int buzz = 2;

const int dt = 6;
const int sck = 7;
float factor_calibracion = 102.82;
HX711_ADC balanza(dt, sck);

void setup() {
  Serial.begin(115200);

  pinMode(boton_emergencia,INPUT);
  pinMode(boton_ignicion, INPUT);
  pinMode(boton_onoff, INPUT);
  pinMode(buzz, OUTPUT);

  SerialPrint::msg("iniciando balanza...");
  balanza.begin();
  delay(500);
  balanza.start(2000);
  // Verificar si la lectura está saturada (típico de cables de celda sueltos)
  if (abs(balanza.getData()) > 100000) {
    SerialPrint::err("[ERR C02]");
  } else {
    delay(500);
    balanza.setCalFactor(-factor_calibracion);
    delay(500);
    balanza.tare();
  }
  SerialPrint::msg("balanza lista");
}

void loop() {
  bool estado_boton_emergencia = digitalRead(boton_emergencia);
  bool estado_boton_ignicion = digitalRead(boton_ignicion);
  bool estado_boton_onoff = digitalRead(boton_onoff);

  if(estado_boton_ignicion == HIGH) {
    digitalWrite(buzz, HIGH);
  } else{
    digitalWrite(buzz, LOW);
  }

  bool nuevo_dato = balanza.update();
  // if (nuevo_dato) { peso = balanza.getData();}
  Serial.print("PLOT$peso=");
  Serial.print(balanza.getData());

  Serial.print(";btn_ign=");
  Serial.print(estado_boton_ignicion);

  Serial.print(";btn_emg=");
  Serial.print(estado_boton_emergencia);

  Serial.print(";btn_onoff=");
  Serial.print(estado_boton_onoff);

  Serial.print(";t_time=");
  Serial.println(millis());
}
