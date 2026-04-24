#pragma once
#include <Arduino.h>
#include "../CC1101/CC1101.h"

// ============================================================
//  Protocole Itron EverBlu Cyble Enhanced
//
//  Séquence :
//   1. TX wake-up : ~2.5s de 0x55 en 2-FSK 2.4 kbps
//   2. TX requête : 39 octets (sync + payload encodé + CRC Kermit)
//   3. RX ack     : 2 phases (sync 0x5550 @ 2.4k → sync 0xFFF0 @ 9.6k), timeout 150 ms
//   4. RX données : 2 phases (idem), timeout 700 ms
//
//  Détection sync par pin GDO0 (IOCFG0=0x06)
//  Contrainte : répond uniquement entre 06:00 et 18:59
//
//  Basé sur le reverse-engineering de hallard/psykokwak
// ============================================================

struct EverBluData {
    uint32_t liters;     // Volume total en litres
    uint8_t  battery;    // Mois de batterie restante
    uint8_t  readCount;  // Compteur de lectures (1–255, rollover)
    int8_t   rssi;       // Niveau du signal (dBm)
    bool     valid;
};

class EverBlu {
public:
    explicit EverBlu(CC1101& radio);

    // Interroge un compteur.
    // serial = 6 chiffres centraux du numéro de module (ex: 356155)
    // year   = 2 derniers chiffres de l'année (ex: 19 pour 2019)
    // Retourne true si une réponse valide est reçue.
    bool request(uint32_t serial, uint8_t year, EverBluData& out);

    // Retourne true si l'heure courante est dans la fenêtre autorisée (06:00–18:59)
    static bool withinTimeWindow();

private:
    CC1101& _radio;

    // ---- Phase TX ------------------------------------------
    void _sendWakeupAndRequest(const uint8_t* reqBuf, uint8_t reqLen);

    // Exécute le remplissage FIFO du wake-up puis envoie reqBuf.
    // Retourne false si un timeout matériel survient (le cleanup TX
    // reste à la charge de _sendWakeupAndRequest).
    bool _doWakeupTX(const uint8_t* reqBuf, uint8_t reqLen);

    // Construit le buffer de requête dans out (39 octets max)
    uint8_t _buildRequest(uint32_t serial, uint8_t year, uint8_t* out);

    // ---- Phase RX ------------------------------------------
    // Réception trame Radian 2 phases : sync 0x5550 @ 2.4k (GDO0)
    // puis sync 0xFFF0 @ 9.6k (GDO0) + lecture données 4×
    int _receiveFrame(uint8_t sizeBytes, uint32_t phase1Ms, uint32_t phase2Ms,
                      uint8_t* buf, uint16_t bufSize);

    // Entrée en RX avec attente MARCSTATE (comme cc1101_rec_mode)
    void _enterRX();

    // ---- Décodage ------------------------------------------
    // Décode un buffer 4× oversampled avec encodage série start/stop
    bool _decodeResponse(const uint8_t* raw, uint16_t rawLen,
                         uint8_t* out, uint8_t& outLen);

    // Extrait les valeurs du buffer décodé
    bool _parseData(const uint8_t* decoded, uint8_t len, EverBluData& out);

    // ---- Encodage série ------------------------------------
    // Encode chaque byte : [start=0][bit0..bit7 LSB-first][stop=111]
    // 12 bits par byte → packed MSB-first dans out[]
    uint8_t _encodeBytes(const uint8_t* in, uint8_t inLen, uint8_t* out);

    // ---- CRC Kermit ----------------------------------------
    // Poly 0x8408, init 0x0000, résultat : bytes swappés
    uint16_t _crcKermit(const uint8_t* data, uint8_t len);

    // ---- Validation CRC réponse ----------------------------
    // Vérifie le CRC Kermit d'une trame décodée.
    // Convention protocole EverBlu : CRC sur les (len-2) premiers
    // octets, stocké aux octets [len-2] et [len-1] (big-endian).
    // Retourne true si valide, false si corruption détectée.
    bool _verifyCrc(const uint8_t* data, uint8_t len);

    // ---- Helpers bits --------------------------------------
    // Lit 1 bit à la position pos (MSB-first) dans buf
    static uint8_t _bit(const uint8_t* buf, uint16_t pos);
    // Lit 4 bits consécutifs et retourne la valeur majoritaire
    static uint8_t _bit4x(const uint8_t* buf, uint16_t pos);
};
