#pragma once

#include <stdint.h>

// =============================================================================
// 123TUNE+ BLE-Notify-Decoder - gemeinsam fuer Hub / M5 / Waveshare.
// Kanonische Quelle: spartan3v2-can-adapter/include/tune123_decode.h
// In die Display-Firmwares (M5 / Waveshare) 1:1 hineinkopieren, damit alle
// denselben Decoder + dieselben Skalierungen verwenden.
//
// Jedes 123-Notify-Paket ist 3 Byte: [opcode][hi_ascii_hex][lo_ascii_hex].
// hi/lo sind ASCII-Hex-Nibbles ('0'..'9','A'..'F'); raw = (hi<<4)|lo.
// (Die $-Handshake-/Keepalive-Logik ist NICHT Teil dieses Headers.)
// =============================================================================

// ASCII-Hex-Nibble -> 0..15 (ungueltige Zeichen -> 0).
inline int tune123HexNibble(uint8_t c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0;
}

// Welcher Cockpit-Wert wurde durch das Paket aktualisiert.
enum class Tune123Field : uint8_t {
  None = 0,      // kein Wert (0x42-ACK oder unbekannter Opcode)
  Rpm,           // U/min
  Advance,       // Grad Fruehzuendung
  Map,           // kPa
  Temperature,   // Grad C (Verteiler)
  CoilCurrent,   // A (Zuendspulenstrom)
  Voltage,       // V (123 intern)
};

struct Tune123Decoded {
  Tune123Field field = Tune123Field::None;  // aktualisierter Wert (None = keiner)
  float value = 0.0f;                        // dekodierter Wert in der Feld-Einheit
  bool known = false;                        // bekannter Opcode (inkl. 0x42-ACK)?
};

// Decodiert opcode + die zwei Hex-Nibbles (hi/lo) in Feld + Wert.
inline Tune123Decoded tune123Decode(uint8_t opcode, int hi, int lo)
{
  const int raw = (hi << 4) | lo;
  Tune123Decoded out;
  out.known = true;
  switch (opcode) {
    case 0x30: out.field = Tune123Field::Rpm;         out.value = hi * 800.0f + lo * 50.0f;    break;
    case 0x31: out.field = Tune123Field::Advance;     out.value = hi * 3.2f + lo * 0.2f;        break;
    case 0x32: out.field = Tune123Field::Map;         out.value = static_cast<float>(raw);      break;
    case 0x33: out.field = Tune123Field::Temperature; out.value = static_cast<float>(raw - 30); break;
    case 0x35: out.field = Tune123Field::CoilCurrent; out.value = raw / 8.65f;                  break;
    case 0x41: out.field = Tune123Field::Voltage;     out.value = raw / 4.54f;                  break;
    case 0x42: /* ACK-artig, kein Wert -> known bleibt true */                                  break;
    default:   out.known = false;                                                               break;
  }
  return out;
}

// Bequem: dekodiert ein komplettes 3-Byte-Notify-Frame direkt.
inline Tune123Decoded tune123DecodeFrame(const uint8_t *data, uint32_t length)
{
  if (length < 3) return Tune123Decoded{};
  return tune123Decode(data[0], tune123HexNibble(data[1]), tune123HexNibble(data[2]));
}
