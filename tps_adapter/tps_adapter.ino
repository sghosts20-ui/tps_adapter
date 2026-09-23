/*
 * ============================================================
 *  TPS Adapter — Нештатный ДПДЗ → Блок управления АКПП
 *  Arduino Nano (АЦП) + PCF8591 (ЦАП)
 *  v1.2 — управление через Serial-терминал
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
 *    v  — живой мониторинг входного напряжения ДПДЗ (бар + направление)
 *    c  — калибровка (2 шага с подсказками)
 *    s  — показать текущую калибровку
 *    r  — сброс в заводские умолчания
 *    h  — помощь
 *
 *  В режиме мониторинга каждые 250 мс выводится:
 *    ADC_in=NNN  TPS=NN.N%  DAC=NNN  Vout≈N.NNV
 * ============================================================
 */

#include <Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// ─── PCF8591 ─────────────────────────────────────────────────
const uint8_t PCF_ADDR = 0x48;

// ─── ВЫХОДНОЙ ДИАПАЗОН ────────────────────────────────────────
// Родной ДПДЗ АКПП: 0.47В закрыта, 4.57В открыта, VREF = 5В
// DAC = V / 5.0 * 255
const uint8_t DAC_CLOSED = 24;   // 0.47 / 5.0 * 255 = 23.97
const uint8_t DAC_OPEN   = 233;  // 4.57 / 5.0 * 255 = 233.07

// ─── EEPROM ───────────────────────────────────────────────────
const uint16_t EE_MAGIC   = 0;   // uint16_t
const uint16_t EE_RCLOSED = 2;   // int16_t
const uint16_t EE_ROPEN   = 4;   // int16_t
const uint16_t MAGIC_VAL  = 0xDA7B;

// ─── КАЛИБРОВОЧНЫЕ ДАННЫЕ ─────────────────────────────────────
int16_t rawClosed = 51;    // умолчание — нормальный датчик
int16_t rawOpen   = 972;   // rawClosed > rawOpen = инвертированный


// ═══════════════════════════════════════════════════════════════
//  ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ═══════════════════════════════════════════════════════════════

int16_t readTPS() {
  uint32_t s = 0;
  for (uint8_t i = 0; i < 8; i++) s += analogRead(A0);
  return (int16_t)(s >> 3);
}

void dacWrite(uint8_t v) {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(0x40);
  Wire.write(v);
  Wire.endTransmission();
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
  // Стираем EEPROM — записываем невалидную сигнатуру
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
  Serial.println(F("    v  — мониторинг Vin ДПДЗ (бар + направление, выход — любая клавиша)"));
  Serial.println(F("    c  — калибровка"));
  Serial.println(F("    s  — показать калибровку"));
  Serial.println(F("    r  — сброс в умолчания"));
  Serial.println(F("    h  — эта справка"));
}

// ═══════════════════════════════════════════════════════════════
//  ЖИВОЙ МОНИТОРИНГ ВХОДНОГО НАПРЯЖЕНИЯ
// ═══════════════════════════════════════════════════════════════

// Показывает входное напряжение ДПДЗ в виде бар-графа и стрелки
// направления — чтобы понять нормальный датчик или инвертированный.
// Выход — любая клавиша.
void showVoltageMonitor() {
  Serial.println(F("\n  Мониторинг Vin ДПДЗ  (любая клавиша — выход)"));
  Serial.println(F("  Двигайте заслонку и смотрите направление стрелки:"));
  Serial.println(F("  нормальный: закрыта=низкое V, открыта=высокое V  (^)"));
  Serial.println(F("  инверсный : закрыта=высокое V, открыта=низкое V  (v)\n"));
  while (Serial.available()) Serial.read();

  int16_t prev   = readTPS();
  uint32_t tNext = 0;

  for (;;) {
    wdt_reset();
    if (Serial.available()) break;

    if (millis() >= tNext) {
      tNext = millis() + 100;

      int16_t raw = readTPS();
      float   v   = (float)raw / 1023.0f * 5.0f;

      // Определяем направление (гистерезис ±3 ед.)
      char dir;
      if      (raw > prev + 3) dir = '^';
      else if (raw < prev - 3) dir = 'v';
      else                     dir = '=';
      prev = raw;

      // Бар-граф 30 символов: заполнение пропорционально raw/1023
      uint8_t fill = (uint32_t)raw * 30 / 1023;

      Serial.print(F("\r  ADC="));
      // выравниваем по 4 разряда
      if (raw < 1000) Serial.print(' ');
      if (raw < 100)  Serial.print(' ');
      if (raw < 10)   Serial.print(' ');
      Serial.print(raw);
      Serial.print(F("  Vin="));
      Serial.print(v, 3);
      Serial.print(F("V  ["));
      for (uint8_t i = 0; i < 30; i++) {
        if (i < fill)       Serial.print('=');
        else if (i == fill) Serial.print('|');
        else                Serial.print(' ');
      }
      Serial.print(F("]  "));
      if      (dir == '^') Serial.print(F("РАСТЁТ ^ "));
      else if (dir == 'v') Serial.print(F("ПАДАЕТ v "));
      else                 Serial.print(F("стоит  = "));
    }
  }

  while (Serial.available()) Serial.read();
  Serial.println(F("\n\n  Выход из мониторинга\n"));
}


// ═══════════════════════════════════════════════════════════════
//  ОЖИДАНИЕ СТРОКИ ИЗ SERIAL (с wdt_reset)
// ═══════════════════════════════════════════════════════════════

// Блокирует выполнение до прихода '\n' или '\r'.
// Всё время сбрасывает watchdog и поддерживает DAC.
// Возвращает первый непробельный символ строки (или '\0').
char waitLine(int16_t *dacRawForKeepAlive = nullptr) {
  // Сбросить буфер
  while (Serial.available()) Serial.read();

  char first = '\0';
  bool gotFirst = false;

  while (true) {
    wdt_reset();

    // Поддерживаем DAC живым
    if (dacRawForKeepAlive) {
      dacWrite(computeDAC(*dacRawForKeepAlive));
    }

    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        if (gotFirst) break;   // получили завершение строки
      } else {
        if (!gotFirst) { first = c; gotFirst = true; }
      }
    }
  }
  // Ждём, пока уйдёт весь возможный мусор (\r\n или \n\r)
  uint32_t t = millis();
  while (millis() - t < 30) {
    wdt_reset();
    while (Serial.available()) Serial.read();
  }
  return first;
}


// ═══════════════════════════════════════════════════════════════
//  КАЛИБРОВКА
// ═══════════════════════════════════════════════════════════════

void runCalibration() {
  Serial.println();
  Serial.println(F("  ┌─ КАЛИБРОВКА ──────────────────────────────┐"));
  Serial.println(F("  │ (нажмите Enter на каждом шаге)             │"));
  Serial.println(F("  └──────────────────────────────────────────────┘"));

  dacWrite(DAC_CLOSED);   // безопасный выход во время калибровки

  // ── Шаг 1: закрытая заслонка ──────────────────────────────
  Serial.println();
  Serial.println(F("  [1/2] Переведите заслонку в ЗАКРЫТОЕ положение"));
  Serial.println(F("        Нажмите Enter когда готово..."));

  // Пока ждём — показываем живые значения в той же строке
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

    // Ждём 200мс или Enter
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

  // ── Шаг 2: полный газ ──────────────────────────────────────
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

  // ── Проверка ───────────────────────────────────────────────
  int16_t span = abs(rOpen - rClose);
  if (span < 50) {
    Serial.print(F("\n  [ОШИБКА] Диапазон = "));
    Serial.print(span);
    Serial.println(F(" ед. — слишком мало! Проверьте подключение."));
    Serial.println(F("  Калибровка НЕ сохранена.\n"));
    return;
  }

  rawClosed = rClose;
  rawOpen   = rOpen;
  saveCalibration();

  Serial.println();
  printCalibration();
  Serial.println(F("  Калибровка завершена!\n"));
}


// ═══════════════════════════════════════════════════════════════
//  ОБРАБОТКА КОМАНД
// ═══════════════════════════════════════════════════════════════

void handleCommand(char cmd) {
  Serial.println();   // новая строка после введённого символа
  switch (cmd) {
    case 'v': case 'V':
      showVoltageMonitor();
      break;
    case 'c': case 'C':
      runCalibration();
      break;
    case 's': case 'S':
      printCalibration();
      break;
    case 'r': case 'R':
      resetCalibration();
      break;
    case 'h': case 'H': case '?':
      printHelp();
      break;
    default:
      Serial.print(F("  Неизвестная команда: "));
      Serial.println(cmd);
      printHelp();
      break;
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
  Serial.println(F("║   TPS Adapter v1.2  (ДПДЗ → АКПП)   ║"));
  Serial.println(F("╚══════════════════════════════════════╝"));

  loadCalibration();
  printCalibration();
  printHelp();
  Serial.println();

  wdt_enable(WDTO_250MS);
}

void loop() {
  wdt_reset();

  // ── Команда из терминала ────────────────────────────────────
  if (Serial.available()) {
    char c = (char)Serial.read();
    // Игнорируем \r, \n и пробелы (мусор после Enter)
    if (c != '\r' && c != '\n' && c != ' ') {
      handleCommand(c);
    }
    return;   // пропустить один цикл мониторинга после команды
  }

  // ── Основное преобразование ─────────────────────────────────
  int16_t raw    = readTPS();
  uint8_t dacVal = computeDAC(raw);
  dacWrite(dacVal);

  // ── Мониторинг каждые 250 мс ────────────────────────────────
  static uint32_t tPrint = 0;
  if (millis() - tPrint >= 250) {
    tPrint = millis();

    int32_t span = (int32_t)rawOpen - rawClosed;
    float pct = (span == 0)
                  ? 0.0f
                  : (float)(raw - rawClosed) * 100.0f / (float)span;
    pct = constrain(pct, 0.0f, 100.0f);
    float vOut = (float)dacVal / 255.0f * 5.0f;

    Serial.print(F("ADC_in="));  Serial.print(raw);
    Serial.print(F("  TPS="));   Serial.print(pct, 1);  Serial.print(F("%"));
    Serial.print(F("  DAC="));   Serial.print(dacVal);
    Serial.print(F("  Vout≈"));  Serial.print(vOut, 2); Serial.println(F("V"));
  }
}
