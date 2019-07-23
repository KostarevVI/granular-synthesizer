#include <avr/io.h>
#include <avr/interrupt.h>
#include <MIDI.h> // Требуется MIDI Library v4.2 или более поздние

#define BENDRANGE 12 //колесо питч-бэнда в полутонах

const bool dipSwitchInstalled = true;
// DIP Switch Setting = Channel (1=вкл, 0=выкл)
// 0000 = 1   0001 = 2   0010 = 3   0011 = 4
// 0100 = 5   0101 = 6   0110 = 7   0111 = 8
// 1000 = 9   1001 = 10  1010 = 11  1011 = 12
// 1100 = 13  1101 = 14  1110 = 15  1111 = 16

#define MIDICHANNEL 1 //Дефолтный мидиканад (1-16)

MIDI_CREATE_DEFAULT_INSTANCE();

uint16_t syncPhaseAcc;
uint16_t syncPhaseInc;
uint16_t grainPhaseAcc;
uint16_t grainPhaseInc;
uint16_t grainAmp;
uint8_t grainDecay;
uint16_t grain2PhaseAcc;
uint16_t grain2PhaseInc;
uint16_t grain2Amp;
uint8_t grain2Decay;

// Пины дип переключателя
#define DIP_SW1 6
#define DIP_SW2 7
#define DIP_SW3 8
#define DIP_SW4 9

//Аналоговые каналы
#define SYNC_CONTROL (4)
#define GRAIN_FREQ_CONTROL (0)
#define GRAIN_DECAY_CONTROL (2)
#define GRAIN2_FREQ_CONTROL (3)
#define GRAIN2_DECAY_CONTROL (1)

#define BUFFER 8 //размер буффера

#define PWM_PIN       3       //порт вывода
#define PWM_VALUE     OCR2B
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       5
#define PWM_INTERRUPT TIMER2_OVF_vect

//Плавный логарифмический маппинг
uint16_t antilogTable[] = {
  64830, 64132, 63441, 62757, 62081, 61413, 60751, 60097, 59449, 58809, 58176, 57549, 56929, 56316, 55709, 55109,
  54515, 53928, 53347, 52773, 52204, 51642, 51085, 50535, 49991, 49452, 48920, 48393, 47871, 47356, 46846, 46341,
  45842, 45348, 44859, 44376, 43898, 43425, 42958, 42495, 42037, 41584, 41136, 40693, 40255, 39821, 39392, 38968,
  38548, 38133, 37722, 37316, 36914, 36516, 36123, 35734, 35349, 34968, 34591, 34219, 33850, 33486, 33125, 32768
};
uint16_t mapPhaseInc(uint16_t input) {
  return (antilogTable[input & 0x3f]) >> (input >> 6);
}

//Маппинг миди нот (роматические частоты)
uint16_t midiTable[] = {
  0, 18, 19, 20, 22, 23, 24, 26, 27, 29, 31, 32, 34, 36, 38, 41, 43, 46, 48, 51, 54, 58, 61, 65, 69, 73,
  77, 82, 86, 92, 97, 103, 109, 115, 122, 129, 137, 145, 154, 163, 173, 183, 194, 206, 218, 231,
  244, 259, 274, 291, 308, 326, 346, 366, 388, 411, 435, 461, 489, 518, 549, 581, 616, 652, 691,
  732, 776, 822, 871, 923, 978, 1036, 1097, 1163, 1232, 1305, 1383, 1465, 1552, 1644, 1742,
  1845, 1955, 2071, 2195, 2325, 2463, 2610, 2765, 2930, 3104, 3288, 3484, 3691, 3910, 4143,
  4389, 4650, 4927, 5220, 5530, 5859, 6207, 6577, 6968, 7382, 7821, 8286, 8779, 9301, 9854,
  10440, 11060, 11718, 12415, 13153, 13935, 14764, 15642, 16572, 17557, 18601, 19708, 20879,
  22121, 23436, 24830, 26306
};
uint16_t mapMidi(uint16_t input) {
  return (midiTable[input]);
}

//Маппинг частот мажорной пентатоники (пентатонической гармонии)
uint16_t pentatonicTable[54] = {
  0, 19, 22, 26, 29, 32, 38, 43, 51, 58, 65, 77, 86, 103, 115, 129, 154, 173, 206, 231, 259, 308, 346,
  411, 461, 518, 616, 691, 822, 923, 1036, 1232, 1383, 1644, 1845, 2071, 2463, 2765, 3288,
  3691, 4143, 4927, 5530, 6577, 7382, 8286, 9854, 11060, 13153, 14764, 16572, 19708, 22121, 26306
};

uint16_t mapPentatonic(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 53);
  return (pentatonicTable[value]);
}

//Глобальные переменные для MIDI
byte note = 0;
byte buff[BUFFER];
byte buffersize = 0;
float bendfactor = ((BENDRANGE * 100.0) / 8190.0);
float cents = 0;
bool pot = false;       //потенциометр был тронут
bool noteOn = false;
int oldpot = 0;
int transpose = 0;

void audioOn() {
  //Запускаем PWM на частоте 31.25kHz
  TCCR2A = _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = _BV(TOIE2);
}

void setup() {
  if (dipSwitchInstalled) {
    pinMode(DIP_SW1, INPUT_PULLUP);
    pinMode(DIP_SW2, INPUT_PULLUP);
    pinMode(DIP_SW3, INPUT_PULLUP);
    pinMode(DIP_SW4, INPUT_PULLUP);
  }
  pinMode(PWM_PIN, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);    //питание 5v
  audioOn();
  pinMode(LED_PIN, OUTPUT);
  oldpot = analogRead(SYNC_CONTROL);
  if (dipSwitchInstalled) {
    MIDI.begin(DipSwitch());
  }
  else {
    MIDI.begin(MIDICHANNEL);
  }
  MIDI.setHandleNoteOn(NoteOnMidi);
  MIDI.setHandleNoteOff(NoteOffMidi);
  MIDI.setHandlePitchBend(Pitchbend);
}

void loop() {

  MIDI.read();

  if (abs(analogRead(SYNC_CONTROL) - oldpot) > 5) pot = true;//Проверяем изменилось ли значение потенциометра SYNC

  //Если значение потенциометра изменилось - Играем мажорную пентатонику с корневой нотой, заданной на миди клавиатуре
  if (pot == true && noteOn == true) {
    syncPhaseInc = (mapPentatonic(analogRead(SYNC_CONTROL)) * pow(2, ((cents + transpose) / 1200)) );
    oldpot = analogRead(SYNC_CONTROL);
  }
  //Если потенциометр SYNC нетронут - играем ноту≤ отправленную с миди клавиатуры
  else {
    syncPhaseInc = mapMidi(note) * pow(2, (cents / 1200));
  }

  grainPhaseInc  = mapPhaseInc(analogRead(GRAIN_FREQ_CONTROL)) / 2;
  grainDecay     = analogRead(GRAIN_DECAY_CONTROL) / 8;
  grain2PhaseInc = mapPhaseInc(analogRead(GRAIN2_FREQ_CONTROL)) / 2;
  grain2Decay    = analogRead(GRAIN2_DECAY_CONTROL) / 4;
}

SIGNAL(PWM_INTERRUPT)
{
  uint8_t value;
  uint16_t output;

  syncPhaseAcc += syncPhaseInc;
  if (syncPhaseAcc < syncPhaseInc) {
    //Начинаем новую гранулу
    grainPhaseAcc = 0;
    grainAmp = 0x7fff;
    grain2PhaseAcc = 0;
    grain2Amp = 0x7fff;
    LED_PORT ^= 1 << LED_BIT; // Аналог digitalWrite но быстрее
  }

  //Увеличиваем фазу осцилятора
  grainPhaseAcc += grainPhaseInc;
  grain2PhaseAcc += grain2PhaseInc;

  //Конвертируем синал в треугольную волну
  value = (grainPhaseAcc >> 7) & 0xff;
  if (grainPhaseAcc & 0x8000) value = ~value;
  //Умножаем на частоту ноты, чтобы получить сэмпл
  output = value * (grainAmp >> 8);

  //Повторяем для второй гранулы
  value = (grain2PhaseAcc >> 7) & 0xff;
  if (grain2PhaseAcc & 0x8000) value = ~value;
  output += value * (grain2Amp >> 8);

  // Каждый семпл создаём задержку
  grainAmp -= (grainAmp >> 8) * grainDecay;
  grain2Amp -= (grain2Amp >> 8) * grain2Decay;

  //Ограничение сигнала дабы не было клиппинга
  output >>= 9;
  if (output > 255) output = 255;

  //Выводим в PWM (так быстрее чем digitalWrite)
  PWM_VALUE = output;
}

//Эта функция вызывается если было зарегисрировано событие нажатой клавиши
//Кладём ноту в буффер и получаем саму ноту
void NoteOnMidi(byte channel, byte pitch, byte velocity) {
  if (buffersize < BUFFER) {
    note = pitch;
    buff[buffersize] = pitch;
    buffersize++;
  }
  noteOn = true;
  //Выбираем ноту пентатоники
  int offset = pitch % 12;
  if (offset < 7) transpose = (5 + offset) * 100;
  else if (offset >= 7) transpose = (offset - 7) * 100;
}

//Эта функция вызывает при окончании нажатия ноты
//Если в буффере остались ноты (больше одной ноты было нажато) - воспроизводим старую ноту
//Если буффер пуст - выбираем 0 ноту (нет звука)
void NoteOffMidi(byte channel, byte pitch, byte velocity) {
  if (buffersize > 1) {
    for (int ctr = 0; ctr < buffersize; ctr++) {
      if (buff[ctr] == pitch) {

        ShiftLeft(ctr + 1);
        break;
      }
    }
    note = buff[buffersize - 1];
    int offset = note % 12;
    if (offset < 7) transpose = (5 + offset) * 100;
    else if (offset >= 7) transpose = (offset - 7) * 100;
  }
  else {
    note = 0;
    noteOn = false;
    buff[buffersize - 1] = 0;
    buffersize = 0;
  }
  pot = false;
}

//Эта функция закрывает пропуски в буффере после удаления ноты
void ShiftLeft(int index) {
  int ctr = (index - 1);
  for (ctr; ctr < buffersize - 1; ctr++) {
    buff[ctr] = buff[ctr + 1];
  }
  buff[ctr + 1] = 0;
  buffersize--;
}

//Вызывается когда получаются данные с колеса питч-бэнда
//Эта функция конвертирует значение колеса в центы (100 цент = 1 полутон)
//В loop() частота ноты + влияние питч-бэнда высчитывается по формуле: конечная частота = нота * 2^(цент/1200)
void Pitchbend (byte channel, int bend) {
  cents = bend * bendfactor;
}

byte DipSwitch() {
  byte value = 0;
  if (digitalRead(DIP_SW4) == LOW)
    value += 1;
  if (digitalRead(DIP_SW3) == LOW)
    value += 2;
  if (digitalRead(DIP_SW2) == LOW)
    value += 4;
  if (digitalRead(DIP_SW1) == LOW)
    value += 8;
  return (value + 1);
}
