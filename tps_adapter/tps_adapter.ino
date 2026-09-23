/*
 * ============================================================
 *  TPS Adapter — Нештатный ДПДЗ → Блок управления АКПП
 *  Arduino Nano (АЦП) + PCF8591 (ЦАП)
 *  v1.3 — управление через Serial-терминал
 * ============================================================
 *
 *  СХЕМА ПОДКЛЮЧЕНИЯ:
 *  ┌─────────────────────────────────────────────────────────┐
 *  │ Arduino A0    → Сигнал нештатного ДПДЗ                  │
 *  │ Arduino A4    → PCF8591 SDA                             │
 *  │ Arduino A5    → PCF8591 SCL                             │
 *  │ Arduino GND   → Общая масса (ДПДЗ + блок АКПП)         │
 *  │                                                         │
 *  │ PCF8591 VDD   → 5V Arduino                              │
 *  │ PCF8591 GND   → Общая масса                             │
 *  │ PCF8591 VREF  → 5V от блока АКПП (питание датчика)     │
 *  │ PCF8591 AOUT  → Вход ДПДЗ на блок АКПП                 │
 *  │ PCF8591 A0-A2 → GND  (I²C адрес 0x48)                  │
 *  └─────────────────────────────────────────────────────────┘
 *
 *  КОМАНДЫ (115200 baud, line ending: Newline):
 *    v   — разово: текущее входное напряжение ДПДЗ
 *    vs  — вкл/выкл потоковый вывод (повторно — выкл)
 *    d   — диагностика PCF8591 / I²C
 *    c   — калибровка (2 шага с подсказками)
 *    s   — показать текущую калибровку
 *    r   — сброс в заводские умолчания
 *    h   — помощь
 * ============================================================
 */

#include <Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <string.h>
#include <ctype.h>

// ─── PCF8591 ─────────────────────────────────────────────────
const uint8_t PCF_ADDR = 0x48;

// ─── ВЫХОДНОЙ ДИАПАЗОН ────────────────────────────────────────
// Родной ДПДЗ АКПП: 0.47В закрыта, 4.57В открыта, VREF = 5В
const uint8_t DAC_CLOSED = 24;   // 0.47 / 5.0 * 255 ≈ 24
const uint8_t DAC_OPEN   = 233;  // 4.57 / 5.0 * 255 ≈ 233

// ─── EEPROM ───────────────────────────────────────────────────
const uint16_t EE_MAGIC   = 0;
const uint16_t EE_RCLOSED = 2;
const uint16_t EE_ROPEN   = 4;
const uint16_t MAGIC_VAL  = 0xDA7B;

// ─── Состояние ────────────────────────────────────────────────
int16_t rawClosed = 51;
int16_t rawOpen   = 972;
bool    streamOn  = false;   // потоковый вывод (команда vs)


// ═══════════════════════════════════════════════════════════════
//  ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ═══════════════════════════════════════════════════════════════

int16_t readTPS() {
  uint32_t s = 0;
  for (uint8_t i = 0; i < 8; i++) s += analogRead(A0);
  return (int16_t)(s >> 3);
}

// Возвращает код endTransmission(): 0 = OK
uint8_t dacWrite(uint8_t v) {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(0x40);
  Wire.write(v);
  return Wire.endTransmission();
}

uint8_t computeDAC(int16_t raw) {
  int32_t span = (int32_t)rawOpen - rawClosed;
  if (span == 0) return DAC_CLOSED;
  int32_t y = (int32_t)(raw - rawClosed)
              * (int32_t)(DAC_OPEN - DAC_CLOSED)
              / span
              + DAC_CLOSED;
  return (uint8_t)constrain(y, (int32_t)DAC_CLOSED, (int32_t)DAC_OPEN);
}

const __FlashStringHelper* i2cErrorStr(uint8_t err) {
  switch (err) {
    case 0: return F("OK");
    case 1: return F("переполнение буфера");
    case 2: return F("NACK на адрес (устройство не отвечает)");
    case 3: return F("NACK на данные");
    case 4: return F("ошибка шины / другое");
    default: return F("неизвестный код");
  }
}


// ═══════════════════════════════════════════════════════════════
//  EEPROM
// ═══════════════════════════════════════════════════════════════

void loadCalibration() {
  uint16_t magic;
  EEPROM.get(EE_MAGIC, magic);
  if (magic != MAGIC_VAL) {
    Serial.println(F("  [EEPROM] Нет данных — загружены умолчания"));
    return;
  }
  EEPROM.get(EE_RCLOSED, rawClosed);
  EEPROM.get(EE_ROPEN,   rawOpen);
  Serial.println(F("  [EEPROM] Калибровка загружена"));
}

void saveCalibration() {
  EEPROM.put(EE_MAGIC,   MAGIC_VAL);
  EEPROM.put(EE_RCLOSED, rawClosed);
  EEPROM.put(EE_ROPEN,   rawOpen);
  Serial.println(F("  [EEPROM] Сохранено"));
}

void resetCalibration() {
  rawClosed = 51;
  rawOpen   = 972;
  EEPROM.put(EE_MAGIC, (uint16_t)0xFFFF);
  Serial.println(F("  Сброшено в умолчания (EEPROM очищен)"));
}


// ═══════════════════════════════════════════════════════════════
//  ВЫВОД СТАТУСА
// ═══════════════════════════════════════════════════════════════

void printCalibration() {
  bool inv = (rawOpen < rawClosed);
  int16_t lo = inv ? rawOpen   : rawClosed;
  int16_t hi = inv ? rawClosed : rawOpen;

  Serial.println(F("  ┌─ Калибровка ──────────────────────────────┐"));
  Serial.print(F("  │ rawClosed = ")); Serial.print(rawClosed);
  Serial.print(F("  ("));
  Serial.print((float)rawClosed / 1023.0f * 5.0f, 2);
  Serial.println(F(" В)                   │"));
  Serial.print(F("  │ rawOpen   = ")); Serial.print(rawOpen);
  Serial.print(F("  ("));
  Serial.print((float)rawOpen / 1023.0f * 5.0f, 2);
  Serial.println(F(" В)                   │"));
  Serial.print(F("  │ Диапазон  = ")); Serial.print(hi - lo);
  Serial.println(F(" ед. АЦП                     │"));
  Serial.print(F("  │ Датчик    = "));
  Serial.println(inv ? F("ИНВЕРТИРОВАННЫЙ              │")
                     : F("НОРМАЛЬНЫЙ                   │"));
  Serial.print(F("  │ Выход:    = "));
  Serial.print((float)DAC_CLOSED / 255.0f * 5.0f, 2);
  Serial.print(F("В ... "));
  Serial.print((float)DAC_OPEN / 255.0f * 5.0f, 2);
  Serial.println(F("В                │"));
  Serial.println(F("  └──────────────────────────────────────────────┘"));
}

void printHelp() {
  Serial.println(F("  Команды:"));
  Serial.println(F("    v   — разово: текущее Vin ДПДЗ"));
  Serial.println(F("    vs  — вкл/выкл потоковый вывод"));
  Serial.println(F("    d   — диагностика PCF8591 / I2C"));
  Serial.println(F("    c   — калибровка"));
  Serial.println(F("    s   — показать калибровку"));
  Serial.println(F("    r   — сброс в умолчания"));
  Serial.println(F("    h   — эта справка"));
}


// ═══════════════════════════════════════════════════════════════
//  v — разовое значение Vin
// ═══════════════════════════════════════════════════════════════

void printVinOnce() {
  int16_t raw = readTPS();
  float   vin = (float)raw / 1023.0f * 5.0f;
  uint8_t dac = computeDAC(raw);
  float   vout = (float)dac / 255.0f * 5.0f;

  int32_t span = (int32_t)rawOpen - rawClosed;
  float pct = (span == 0)
                ? 0.0f
                : (float)(raw - rawClosed) * 100.0f / (float)span;
  pct = constrain(pct, 0.0f, 100.0f);

  Serial.print(F("  ADC="));  Serial.print(raw);
  Serial.print(F("  Vin="));  Serial.print(vin, 3); Serial.print(F("V"));
  Serial.print(F("  TPS="));  Serial.print(pct, 1); Serial.print(F("%"));
  Serial.print(F("  DAC="));  Serial.print(dac);
  Serial.print(F("  Vout≈")); Serial.print(vout, 2); Serial.println(F("V"));
}


// ═══════════════════════════════════════════════════════════════
//  d — диагностика PCF8591
// ═══════════════════════════════════════════════════════════════

void runDiagnostics() {
  Serial.println(F("  ┌─ Диагностика PCF8591 / I2C ───────────────┐"));

  // 1) Скан шины 0x08..0x77
  Serial.println(F("  │ Скан I2C...                                │"));
  uint8_t found = 0;
  bool    pcfFound = false;
  uint8_t pcfAlt = 0;   // другой адрес семейства 0x48..0x4B

  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    wdt_reset();
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      found++;
      Serial.print(F("  │   найден: 0x"));
      if (addr < 0x10) Serial.print('0');
      Serial.print(addr, HEX);
      if (addr == PCF_ADDR) {
        Serial.print(F("  ← PCF8591 (ожидаемый)"));
        pcfFound = true;
      } else if (addr >= 0x48 && addr <= 0x4B) {
        Serial.print(F("  ← возможен PCF8591 (A0-A2)"));
        pcfAlt = addr;
      }
      Serial.println();
      Serial.print(F("  │"));
      // выравнивание строки таблицы — просто перевод
      // (следующий println уже есть выше; печатаем пустую для формата)
    }
  }

  if (found == 0) {
    Serial.println(F("  │   устройств не найдено                     │"));
    Serial.println(F("  │   Проверьте: SDA/SCL, питание, GND, pull-up │"));
  } else {
    Serial.print(F("  │ Всего на шине: "));
    Serial.print(found);
    Serial.println(F(" устр.                        │"));
  }

  // 2) Проверка ожидаемого адреса
  Serial.print(F("  │ Адрес 0x48: "));
  Wire.beginTransmission(PCF_ADDR);
  uint8_t errProbe = Wire.endTransmission();
  if (errProbe == 0) {
    Serial.println(F("ОБНАРУЖЕН                    │"));
  } else {
    Serial.print(F("НЕ ОБНАРУЖЕН ("));
    Serial.print(i2cErrorStr(errProbe));
    Serial.println(F(") │"));
    if (pcfAlt) {
      Serial.print(F("  │ Подсказка: чип на 0x"));
      Serial.print(pcfAlt, HEX);
      Serial.println(F(" — проверьте A0-A2 │"));
    }
  }

  // 3) Тест записи ЦАП
  if (errProbe == 0) {
    uint8_t errW = dacWrite(DAC_CLOSED);
    Serial.print(F("  │ Запись ЦАП (DAC="));
    Serial.print(DAC_CLOSED);
    Serial.print(F("): "));
    if (errW == 0) Serial.println(F("OK                       │"));
    else {
      Serial.print(i2cErrorStr(errW));
      Serial.println(F(" │"));
    }
  } else {
    Serial.println(F("  │ Запись ЦАП: пропущена (нет чипа)          │"));
  }

  // 4) Вход ДПДЗ
  int16_t raw = readTPS();
  float   vin = (float)raw / 1023.0f * 5.0f;
  Serial.print(F("  │ Вход A0: ADC="));
  Serial.print(raw);
  Serial.print(F("  Vin="));
  Serial.print(vin, 3);
  Serial.println(F("V                │"));

  if (raw < 5)
    Serial.println(F("  │   ⚠ сигнал почти 0В — обрыв / нет питания? │"));
  else if (raw > 1015)
    Serial.println(F("  │   ⚠ сигнал почти 5В — КЗ на +5В?          │"));

  Serial.print(F("  │ Поток (vs): "));
  Serial.println(streamOn ? F("ВКЛ                         │")
                          : F("ВЫКЛ                        │"));
  Serial.println(F("  └──────────────────────────────────────────────┘"));
}


// ═══════════════════════════════════════════════════════════════
//  КАЛИБРОВКА
// ═══════════════════════════════════════════════════════════════

void runCalibration() {
  bool wasStream = streamOn;
  streamOn = false;

  Serial.println();
  Serial.println(F("  ┌─ КАЛИБРОВКА ──────────────────────────────┐"));
  Serial.println(F("  │ (нажмите Enter на каждом шаге)             │"));
  Serial.println(F("  └──────────────────────────────────────────────┘"));

  dacWrite(DAC_CLOSED);

  // ── Шаг 1 ──────────────────────────────────────────────────
  Serial.println();
  Serial.println(F("  [1/2] Переведите заслонку в ЗАКРЫТОЕ положение"));
  Serial.println(F("        Нажмите Enter когда готово..."));

  while (Serial.available()) Serial.read();
  while (true) {
    wdt_reset();
    int16_t live = readTPS();
    Serial.print(F("\r        ADC="));
    Serial.print(live);
    Serial.print(F("  ("));
    Serial.print((float)live / 1023.0f * 5.0f, 3);
    Serial.print(F("V)   "));
    dacWrite(DAC_CLOSED);

    uint32_t t = millis();
    while (millis() - t < 200) {
      wdt_reset();
      if (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') goto step1_done;
      }
    }
  }
  step1_done:;
  int16_t rClose = readTPS();
  Serial.println();
  Serial.print(F("  >> rawClosed = ")); Serial.print(rClose);
  Serial.print(F("  (")); Serial.print((float)rClose / 1023.0f * 5.0f, 3);
  Serial.println(F("V)"));

  // ── Шаг 2 ──────────────────────────────────────────────────
  Serial.println();
  Serial.println(F("  [2/2] Переведите заслонку в ПОЛНОЕ ОТКРЫТИЕ"));
  Serial.println(F("        Нажмите Enter когда готово..."));

  while (Serial.available()) Serial.read();
  while (true) {
    wdt_reset();
    int16_t live = readTPS();
    Serial.print(F("\r        ADC="));
    Serial.print(live);
    Serial.print(F("  ("));
    Serial.print((float)live / 1023.0f * 5.0f, 3);
    Serial.print(F("V)   "));
    dacWrite(DAC_CLOSED);

    uint32_t t = millis();
    while (millis() - t < 200) {
      wdt_reset();
      if (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') goto step2_done;
      }
    }
  }
  step2_done:;
  int16_t rOpen = readTPS();
  Serial.println();
  Serial.print(F("  >> rawOpen   = ")); Serial.print(rOpen);
  Serial.print(F("  (")); Serial.print((float)rOpen / 1023.0f * 5.0f, 3);
  Serial.println(F("V)"));

  int16_t span = abs(rOpen - rClose);
  if (span < 50) {
    Serial.print(F("\n  [ОШИБКА] Диапазон = "));
    Serial.print(span);
    Serial.println(F(" ед. — слишком мало! Проверьте подключение."));
    Serial.println(F("  Калибровка НЕ сохранена.\n"));
    streamOn = wasStream;
    return;
  }

  rawClosed = rClose;
  rawOpen   = rOpen;
  saveCalibration();

  Serial.println();
  printCalibration();
  Serial.println(F("  Калибровка завершена!\n"));
  streamOn = wasStream;
}


// ═══════════════════════════════════════════════════════════════
//  ОБРАБОТКА КОМАНД (строка целиком)
// ═══════════════════════════════════════════════════════════════

void handleCommand(char *cmd) {
  // нижний регистр
  for (char *p = cmd; *p; p++) *p = (char)tolower((unsigned char)*p);

  if (strcmp(cmd, "v") == 0) {
    printVinOnce();
  }
  else if (strcmp(cmd, "vs") == 0) {
    streamOn = !streamOn;
    Serial.print(F("  Потоковый вывод: "));
    Serial.println(streamOn ? F("ВКЛ") : F("ВЫКЛ"));
  }
  else if (strcmp(cmd, "d") == 0) {
    runDiagnostics();
  }
  else if (strcmp(cmd, "c") == 0) {
    runCalibration();
  }
  else if (strcmp(cmd, "s") == 0) {
    printCalibration();
  }
  else if (strcmp(cmd, "r") == 0) {
    resetCalibration();
  }
  else if (strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0) {
    printHelp();
  }
  else {
    Serial.print(F("  Неизвестная команда: "));
    Serial.println(cmd);
    printHelp();
  }
}


// ═══════════════════════════════════════════════════════════════
//  SETUP / LOOP
// ═══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  dacWrite(DAC_CLOSED);

  Serial.println(F("\n╔══════════════════════════════════════╗"));
  Serial.println(F("║   TPS Adapter v1.3  (ДПДЗ → АКПП)   ║"));
  Serial.println(F("╚══════════════════════════════════════╝"));

  loadCalibration();
  printCalibration();
  printHelp();
  Serial.println(F("  (поток выкл — включить: vs)\n"));

  wdt_enable(WDTO_250MS);
}

void loop() {
  wdt_reset();

  // ── Чтение строки команды до \n / \r ────────────────────────
  static char   cmdBuf[16];
  static uint8_t cmdLen = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        Serial.println();
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (c != ' ' && cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }

  // ── Основное преобразование (всегда) ────────────────────────
  int16_t raw    = readTPS();
  uint8_t dacVal = computeDAC(raw);
  dacWrite(dacVal);

  // ── Потоковый вывод (только если vs включён) ───────────────
  if (!streamOn) return;

  static uint32_t tPrint = 0;
  if (millis() - tPrint >= 250) {
    tPrint = millis();

    int32_t span = (int32_t)rawOpen - rawClosed;
    float pct = (span == 0)
                  ? 0.0f
                  : (float)(raw - rawClosed) * 100.0f / (float)span;
    pct = constrain(pct, 0.0f, 100.0f);
    float vin  = (float)raw / 1023.0f * 5.0f;
    float vOut = (float)dacVal / 255.0f * 5.0f;

    Serial.print(F("ADC="));   Serial.print(raw);
    Serial.print(F("  Vin=")); Serial.print(vin, 3); Serial.print(F("V"));
    Serial.print(F("  TPS=")); Serial.print(pct, 1); Serial.print(F("%"));
    Serial.print(F("  DAC=")); Serial.print(dacVal);
    Serial.print(F("  Vout≈")); Serial.print(vOut, 2); Serial.println(F("V"));
  }
}
