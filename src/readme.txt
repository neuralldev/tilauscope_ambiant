structure of packets
header,uint16_t,2 bytes,0x5555
temp_x10,int16_t,2 bytes,Température ×10
hum_x10,int16_t,2 bytes,Humidité ×10
press_x10,int32_t,4 bytes,Pression ×10
checksum,uint8_t,1 byte,
footer,uint16_t,2 bytes,0xAAAA

smaple code to unpack in python

import struct

# --- Simulation du Payload Reçu ---
# Exemple de données binaires (13 octets) envoyées par l'ESP32.
# Le format struct sera: < H h h i B H
# < : Little-Endian
# H : unsigned short (uint16_t)
# h : short (int16_t) - pour temp_x10 et hum_x10
# i : int (int32_t) - pour press_x10
# B : unsigned char (uint8_t)
# Note : Les types 'h' et 'i' sont utilisés pour gérer les valeurs signées (int16_t, int32_t)

# --- Format du pack: < H h h i B H
# H(header, 2B) | h(temp, 2B) | h(hum, 2B) | i(press, 4B) | B(checksum, 1B) | H(footer, 2B)

# Création d'un exemple de payload simulé :
# Header: 0x5555 (Little-Endian)
# Temp: 25.3°C -> 253 (0xFD 0x00 Little-Endian)
# Hum: 55.7% -> 557 (0x2D 0x02 Little-Endian)
# Press: 1013.2 hPa -> 10132 (0xB4 0x27 0x00 0x00 Little-Endian)
# Checksum: (0xFD+0x00+0x2D+0x02+0xB4+0x27+0x00+0x00) % 256 = 0x1E
# Footer: 0xAAAA (Little-Endian)
payload_brut = b'\x55\x55\xFD\x00\x2D\x02\xB4\x27\x00\x00\x1E\xAA\xAA'

# --- Décodage du Payload ---
def decodage_payload_ble(payload):
    """
    Dépaquette le payload binaire en Little-Endian et convertit les valeurs en float.
    """
    # 'H': uint16_t, 'h': int16_t, 'i': int32_t, 'B': uint8_t
    # Format: Little-Endian (<) suivi des types de champs
    FORMAT = '<H h h i B H' 

    # Vérification de la taille du payload
    if len(payload) != struct.calcsize(FORMAT):
        raise ValueError(f"Taille de payload invalide. Attendu {struct.calcsize(FORMAT)} octets, reçu {len(payload)}.")
        
    # Dépaquetage
    data_tuple = struct.unpack(FORMAT, payload)
    
    # Extraction des champs dépaquetés
    header, temp_x10, hum_x10, press_x10, checksum, footer = data_tuple

    # --- Vérification de l'intégrité (Checksum) ---
    # Le payload utile pour le checksum commence après le header (byte 2)
    data_payload = payload[2:10]
    calculated_checksum = sum(data_payload) % 256
    
    is_valid = (checksum == calculated_checksum)

    # --- Conversion en Float (Division par 10.0) ---
    temp_float = temp_x10 / 10.0
    hum_float = hum_x10 / 10.0
    press_float = press_x10 / 10.0
    
    # Retourne les valeurs utiles et le statut de validité
    return {
        "temp_c": temp_float,
        "hum_pct": hum_float,
        "press_hpa": press_float,
        "is_valid": is_valid,
        "checksum_recv": checksum,
        "checksum_calc": calculated_checksum
    }

# --- Exécution ---
try:
    resultats = decodage_payload_ble(payload_brut)
    
    print("✅ Données décodées :")
    print(f"  - Température : {resultats['temp_c']:.1f} °C")
    print(f"  - Humidité    : {resultats['hum_pct']:.1f} %")
    print(f"  - Pression    : {resultats['press_hpa']:.1f} hPa")
    print("-" * 30)
    print(f"  - Checksum reçu: 0x{resultats['checksum_recv']:02X}")
    print(f"  - Checksum calculé: 0x{resultats['checksum_calc']:02X}")
    print(f"  - Intégrité : {'VALIDE' if resultats['is_valid'] else 'INVALIDE'}")

except ValueError as e:
    print(f"Erreur de décodage: {e}")