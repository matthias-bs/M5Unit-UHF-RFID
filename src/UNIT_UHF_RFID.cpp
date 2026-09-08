/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "UNIT_UHF_RFID.h"
#include "CMD.h"

String hex2str(uint8_t num)
{
    if (num > 0xf)
    {
        return String(num, HEX);
    }
    else
    {
        return ("0" + String(num, HEX));
    }
}

uint8_t calculateChecksum(const uint8_t *data, size_t offset, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = offset; i < offset + length; i++) {
        checksum += data[i];
    }
    return checksum;
}

/*! @brief Initialize the Unit UHF_RFID.*/
void Unit_UHF_RFID::begin(HardwareSerial *serial, int baud, uint8_t RX, uint8_t TX, bool debug)
{
    _debug = debug;
    _serial = serial;
    _serial->begin(baud, SERIAL_8N1, RX, TX);
}

/*! @brief Clear the buffer.*/
void Unit_UHF_RFID::cleanBuffer()
{
    for (size_t i = 0; i < sizeof(buffer); i++)
    {
        buffer[i] = 0;
    }
}

/*! @brief Clear the card's data buffer.*/
void Unit_UHF_RFID::cleanCardsBuffer()
{
    for (int i = 0; i < 200; i++)
    {
        cards[i] = CARD{0, {0, 0}, {0}, "", "", ""};
    }
}

/*! @brief Waiting for a period of time to receive a message
    @return True if the message is available at the specified time and in the
   specified range, otherwise false..*/
bool Unit_UHF_RFID::waitMsg(unsigned long time)
{
    unsigned long start = millis();
    size_t i = 0;
    cleanBuffer();
    while (_serial->available() || (millis() - start) < time)
    {
        if (_serial->available())
        {
            if (i >= sizeof(buffer))
            {
                break;
            }
            uint8_t b = _serial->read();
            buffer[i] = b;
            i++;
            if (b == 0x7e)
            {
                break;
            }
        }
        else
        {
            yield();
        }
    }
    if (i > 0 && buffer[0] == 0xbb && buffer[i - 1] == 0x7e)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*! @brief Send command.*/
void Unit_UHF_RFID::sendCMD(uint8_t *data, size_t size)
{
    _serial->write(data, size);
}

/*! @brief Print response for debugging */
void Unit_UHF_RFID::debugFrame(const char *fn, size_t len, bool is_cmd)
{
    if (_debug)
    {
        Serial.print("[");
        Serial.print(fn);
        Serial.println("]");
        if (is_cmd)
        {
            Serial.print(">> ");
        }
        else
        {
            Serial.print("<< ");
        }

        const size_t n = (len > sizeof(buffer)) ? sizeof(buffer) : len;
        for (size_t i = 0; i < n; i++)
        {
            Serial.print(hex2str(buffer[i]));
        }
        Serial.println();
        Serial.println("-------------------------");
}

/*! @brief Filter the received message.*/
bool Unit_UHF_RFID::filterCardInfo(String epc)
{
    for (int i = 0; i < 200; i++)
    {
        if (epc == cards[i].epc_str)
        {
            return false;
        }
    }
    return true;
}

/*! @brief Save the card information.*/
bool Unit_UHF_RFID::saveCardInfo(CARD *card)
{
    String rssi = buffer[5] > 0x7f ? "-" + String(256 - buffer[5]) : String(buffer[5]);
    String pc = hex2str(buffer[6]) + hex2str(buffer[7]);
    String epc = "";

    for (uint8_t i = 8; i < 20; i++)
    {
        epc += hex2str(buffer[i]);
    }

    if (!filterCardInfo(epc))
    {
        return false;
    }

    for (uint8_t i = 8; i < 20; i++)
    {
        card->epc[i - 8] = buffer[i];
    }

    card->rssi = buffer[5];
    card->pc[0] = buffer[6];
    card->pc[1] = buffer[7];

    card->rssi_str = rssi;
    card->pc_str = pc;
    card->epc_str = epc;

    if (_debug)
    {
        Serial.println("pc: " + pc);
        Serial.println("rssi: " + rssi);
        Serial.println("epc: " + epc);
        for (uint8_t i = 0; i < 24; i++)
        {
            Serial.print(hex2str(buffer[i]));
        }
        Serial.println(" ");
    }
    return true;
}

/*! @brief Put the module into sleep mode. */
bool Unit_UHF_RFID::sleep()
{
    sendCMD((uint8_t *)SLEEP_CMD, sizeof(SLEEP_CMD));
    if (waitMsg())
    {
        debugFrame(__FUNCTION__, 8);

        if (buffer[2] == SLEEP_CMD[2])
        {
            return true;
        }
    }
    return false;
}

/*! @brief Wake up the module by sending an arbitrary byte. */
void Unit_UHF_RFID::wakeup()
{
    _serial->write(0xFF);
}

uint8_t Unit_UHF_RFID::pollingOnce()
{
    cleanCardsBuffer();
    sendCMD((uint8_t *)POLLING_ONCE_CMD, sizeof(POLLING_ONCE_CMD));
    uint8_t count = 0;
    while (waitMsg())
    {
        if (buffer[23] == 0x7e)
        {
            if (count < 200)
            {
                if (saveCardInfo(&cards[count]))
                {
                    count++;
                }
            }
            else
            {
                return 200;
            }
        }
    }
    return count;
}

uint8_t Unit_UHF_RFID::pollingMultiple(uint16_t polling_count)
{
    cleanCardsBuffer();
    memcpy(buffer, POLLING_MULTIPLE_CMD, sizeof(POLLING_MULTIPLE_CMD));
    buffer[6] = (polling_count >> 8) & 0xff;
    buffer[7] = (polling_count) & 0xff;

    uint8_t check = 0;
    for (uint8_t i = 1; i < 8; i++)
    {
        check += buffer[i];
    }

    buffer[8] = check & 0xff;

    debugFrame(__FUNCTION__, sizeof(POLLING_MULTIPLE_CMD), true);

    sendCMD(buffer, sizeof(POLLING_MULTIPLE_CMD));

    uint8_t count = 0;
    while (waitMsg())
    {
        if (buffer[23] == 0x7e)
        {
            if (count < 200)
            {
                if (saveCardInfo(&cards[count]))
                {
                    count++;
                }
            }
            else
            {
                break;
            }
        }
    }
    return count;
}

/*! @brief Get hardware version information.*/
String Unit_UHF_RFID::getVersion()
{
    sendCMD((uint8_t *)HARDWARE_VERSION_CMD, sizeof(HARDWARE_VERSION_CMD));
    if (waitMsg())
    {
        String info;
        for (uint8_t i = 0; i < 50; i++)
        {
            info += (char)buffer[6 + i];
            if (buffer[8 + i] == 0x7e)
            {
                break;
            }
        }
        return info;
    }
    else
    {
        return "ERROR";
    }
}

/*! @brief Get hardware version information.*/
bool Unit_UHF_RFID::getVersion(String &version)
{
    sendCMD((uint8_t *)HARDWARE_VERSION_CMD, sizeof(HARDWARE_VERSION_CMD));
    if (waitMsg())
    {
        version = "";
        for (uint8_t i = 0; i < 50; i++)
        {
            version += (char)buffer[6 + i];
            if (buffer[8 + i] == 0x7e)
            {
                break;
            }
        }
        if (buffer[2] == HARDWARE_VERSION_CMD[2])
        {
            return true;
        }
    }
    return false;
}

/*! @brief Get the current operating region. */
bool Unit_UHF_RFID::getOperatingRegion(uint8_t &region)
{
    sendCMD((uint8_t *)GET_OPERATING_REGION_CMD, sizeof(GET_OPERATING_REGION_CMD));
    if (waitMsg())
    {
        debugFrame(__FUNCTION__);

        const uint8_t responseType = 0x01;
        const uint16_t payloadLength = (static_cast<uint16_t>(buffer[3]) << 8) | buffer[4];
        const uint8_t responseChecksum = calculateChecksum(buffer, 1, 5);
        const uint8_t receivedRegion = buffer[5];

        if ((buffer[1] == responseType) && (buffer[2] == GET_OPERATING_REGION_CMD[2]) &&
            (payloadLength == 1) && (buffer[6] == responseChecksum) && (receivedRegion != 0) &&
            (receivedRegion != 5) && (receivedRegion <= 6))
        {
            region = receivedRegion;
            return true;
        }
    }
    return false;
}

/*! @brief Set the current operating region. */
bool Unit_UHF_RFID::setOperatingRegion(uint8_t region)
{
    if ((region == 0) || (region == 5) || (region > 6))
    {
        return false;
    }

    memcpy(buffer, SET_OPERATING_REGION_CMD, sizeof(SET_OPERATING_REGION_CMD));
    buffer[5] = region;

    buffer[6] = calculateChecksum(buffer, 1, 5);

    sendCMD(buffer, sizeof(SET_OPERATING_REGION_CMD));
    if (waitMsg())
    {
        debugFrame(__FUNCTION__);

        if (buffer[2] == SET_OPERATING_REGION_CMD[2])
        {
            return true;
        }
    }
    return false;
}

const uint8_t MIXER_G[] = {0, 3, 6, 9, 12, 15, 16};
const uint8_t IF_G[] = {12, 18, 21, 24, 27, 30, 36, 40};

bool Unit_UHF_RFID::getRxDemodParams(uint8_t &mixer_g, uint8_t &if_g, int16_t &thrd)
{
    sendCMD((uint8_t *)GET_RX_DEMOD_PARAMS_CMD, sizeof(GET_RX_DEMOD_PARAMS_CMD));
    if (waitMsg())
    {
        debugFrame(__FUNCTION__);

        if (buffer[2] == GET_RX_DEMOD_PARAMS_CMD[2])
        {
            mixer_g = buffer[5];
            if_g = buffer[6];
            thrd = (buffer[7] << 8) | buffer[8];
            
            if ((mixer_g > 6) || (if_g > 7))
            {
                return false;
            }
            mixer_g = MIXER_G[mixer_g];
            if_g = IF_G[if_g];
            return true;
        }
    }
    return false;
}

bool Unit_UHF_RFID::setRxDemodParams(uint8_t mixer_g, uint8_t if_g, int16_t thrd)
{
    memcpy(buffer, SET_RX_DEMOD_PARAMS_CMD, sizeof(SET_RX_DEMOD_PARAMS_CMD));

    uint8_t mixer_g_index = 0xFF;
    for (uint8_t i = 0; i < 7; i++)
    {
        if (MIXER_G[i] == mixer_g)
        {
            mixer_g_index = i;
            break;
        }
    }
    buffer[5] = mixer_g_index;

    uint8_t if_g_index = 0xFF;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (IF_G[i] == if_g)
        {
            if_g_index = i;
            break;
        }
    }

    if (mixer_g_index == 0xFF || if_g_index == 0xFF)
    {
        // Invalid mixer_g or if_g value
        return false;
    }

    buffer[6] = if_g_index;

    buffer[7] = (thrd >> 8) & 0xff;
    buffer[8] = thrd & 0xff;

    uint8_t check = 0;
    for (uint8_t i = 1; i < 9; i++)
    {
        check += buffer[i];
    }

    buffer[9] = check & 0xff;

    sendCMD(buffer, sizeof(SET_RX_DEMOD_PARAMS_CMD));
    if (waitMsg())
    {
        debugFrame(__FUNCTION__);

        if (buffer[2] == SET_RX_DEMOD_PARAMS_CMD[2])
        {
            return true;
        }
    }
    return false;
}

String Unit_UHF_RFID::selectInfo()
{
    sendCMD((uint8_t *)GET_SELECT_PARAMETER_CMD, sizeof(GET_SELECT_PARAMETER_CMD));
    if (waitMsg())
    {
        String Info = "";
        for (uint8_t i = 12; i < 24; i++)
        {
            Info += hex2str(buffer[i]);
        }
        debugFrame(__FUNCTION__);
        return Info;
    }
    return "ERROR";
}

bool Unit_UHF_RFID::select(uint8_t *epc)
{
    memcpy(buffer, SET_SELECT_PARAMETER_CMD, sizeof(SET_SELECT_PARAMETER_CMD));

    uint8_t check = 0;

    for (uint8_t i = 12; i < 24; i++)
    {
        buffer[i] = epc[i - 12];
    }

    for (uint8_t i = 1; i < 24; i++)
    {
        check += buffer[i];
    }

    buffer[24] = check & 0xff;

    debugFrame(__FUNCTION__, sizeof(SET_SELECT_PARAMETER_CMD), true);

    sendCMD(buffer, sizeof(SET_SELECT_PARAMETER_CMD));
    if (waitMsg())
    {
        debugFrame(__FUNCTION__, 25);

        for (uint8_t i = 0; i < sizeof(SET_SELECT_OK); i++)
        {
            if (SET_SELECT_OK[i] != buffer[i])
            {
                return false;
            }
        }
        return true;
    }
    return false;
}

bool Unit_UHF_RFID::writeCard(uint8_t *data, size_t size, uint8_t membank, uint16_t sa, uint32_t access_password)
{
    if (size > sizeof(buffer) - 16)
    {
        return false;
    }

    memcpy(buffer, WRITE_STORAGE_CMD, sizeof(WRITE_STORAGE_CMD));
    buffer[5] = (access_password >> 24) & 0xff;
    buffer[6] = (access_password >> 16) & 0xff;
    buffer[7] = (access_password >> 8) & 0xff;
    buffer[8] = access_password & 0xff;

    buffer[9] = membank;

    buffer[10] = (sa >> 8) & 0xff;
    buffer[11] = sa & 0xff;

    uint8_t word = size / 2;

    buffer[12] = (word >> 8) & 0xff;
    buffer[13] = word & 0xff;

    size_t offset = 14;
    for (size_t i = 0; i < size; i++)
    {
        buffer[14 + i] = data[i];
        offset++;
    }
    uint8_t check = 0;
    for (uint8_t i = 1; i < offset; i++)
    {
        check += buffer[i];
    }

    buffer[offset] = check & 0xff;
    buffer[offset + 1] = 0x7e;

    debugFrame(__FUNCTION__, offset + 2, true);

    sendCMD(buffer, offset + 2);
    if (waitMsg())
    {
        debugFrame(__FUNCTION__);

        if (WRITE_STORAGE_ERROR[2] == buffer[2])
        {
            Serial.println("Write Error");
            return false;
        }
        return true;
    }
    return false;
}

bool Unit_UHF_RFID::readCard(uint8_t *data, size_t size, uint8_t membank, uint16_t sa, uint32_t access_password)
{
    memcpy(buffer, READ_STORAGE_CMD, sizeof(READ_STORAGE_CMD));
    buffer[5] = (access_password >> 24) & 0xff;
    buffer[6] = (access_password >> 16) & 0xff;
    buffer[7] = (access_password >> 8) & 0xff;
    buffer[8] = access_password & 0xff;
    buffer[9] = membank;
    buffer[10] = (sa >> 8) & 0xff;
    buffer[11] = sa & 0xff;
    uint8_t word = size / 2;
    buffer[12] = (word >> 8) & 0xff;
    buffer[13] = word & 0xff;

    uint8_t check = 0;

    for (uint8_t i = 1; i < 14; i++)
    {
        check += buffer[i];
    }

    buffer[14] = check & 0xff;

    debugFrame(__FUNCTION__, sizeof(READ_STORAGE_CMD), true);

    sendCMD(buffer, sizeof(READ_STORAGE_CMD));
    if (waitMsg())
    {
        debugFrame(__FUNCTION__, 22 + size);

        if (READ_STORAGE_ERROR[2] == buffer[2])
        {
            return false;
        }

        if (size > sizeof(buffer) - 20)
        {
            return false;
        }
        memcpy(data, buffer + 20, size);
        return true;
    }
    return false;
}

bool Unit_UHF_RFID::lockCard(uint32_t flags, uint32_t access_password)
{
    memcpy(buffer, LOCK_STORAGE_CMD, sizeof(LOCK_STORAGE_CMD));
    buffer[5] = (access_password >> 24) & 0xff;
    buffer[6] = (access_password >> 16) & 0xff;
    buffer[7] = (access_password >> 8) & 0xff;
    buffer[8] = access_password & 0xff;
    buffer[9] = (flags >> 16) & 0xff;
    buffer[10] = (flags >> 8) & 0xff;
    buffer[11] = flags & 0xff;

    uint8_t check = 0;

    for (uint8_t i = 1; i < 12; i++)
    {
        check += buffer[i];
    }

    buffer[12] = check & 0xff;

    debugFrame(__FUNCTION__, sizeof(LOCK_STORAGE_CMD), true);

    sendCMD(buffer, sizeof(LOCK_STORAGE_CMD));
    if (waitMsg())
    {
        debugFrame(__FUNCTION__);

        if (buffer[2] == LOCK_STORAGE_CMD[2])
        {
            return true;
        }
    }
    return false;
}

// 2600 => 26dB
bool Unit_UHF_RFID::setTxPower(uint16_t db)
{
    memcpy(buffer, SET_TX_POWER, sizeof(SET_TX_POWER));
    buffer[5] = (db >> 8) & 0xff;
    buffer[6] = db & 0xff;

    uint8_t check = 0;

    for (uint8_t i = 1; i < 7; i++)
    {
        check += buffer[i];
    }
    buffer[7] = check & 0xff;

    debugFrame(__FUNCTION__, sizeof(SET_TX_POWER), true);

    sendCMD(buffer, sizeof(SET_TX_POWER));
    if (waitMsg())
    {
        debugFrame(__FUNCTION__);
        if (buffer[2] == SET_TX_POWER[2])
        {
            return true;
        }
    }
    return false;
}
