#pragma once

namespace aquarium {

static const uint8_t AQUARIUM_COMMAND_GET_PPM = 0xFF;
static const uint16_t AQUARIUM_PREAMBLE = 0x86FF;

uint8_t AQUARIUM_checksum(const uint8_t *command, uint8_t count);

typedef struct __attribute__((__packed__)) 
{
	uint8_t command;
	uint8_t crc;

    void setCRC() { crc = AQUARIUM_checksum((uint8_t *)this, sizeof(*this)-1); }
    bool checkCRC() { return crc == AQUARIUM_checksum((uint8_t *)this, sizeof(*this)-1); }
} CommandContainer;

static const uint8_t AQUARIUM_REQUEST_LENGTH = sizeof(CommandContainer);

typedef struct __attribute__((__packed__)) 
{
	uint16_t sign;
	uint8_t pH;
	uint8_t light;
	uint8_t level;
	uint8_t temperature;
	uint8_t crc;

private:
    uint8_t pack(float v, float scale, float offs)    {        return (uint8_t)round((v - offs) * scale) ;    }
    float unpack(uint8_t v, float scale, float offs)  {        return float(v) / scale + offs;        }

public:
    void setPH(float v)             { pH = pack(v, 10, 0);    }
    void setLight(float v)          { light = pack(v, 0.5, 0);    }
    void setLevel(float v)          { level = pack(v, 2, 0);    }
    void setTemperature(float v)    { temperature = pack(v, 10, 10);    }

    float getPH()             {  return unpack(pH, 10, 0);    }
    float getLight()          {  return unpack(light, 0.5, 0);    }
    float getLevel()          {  return unpack(level, 2, 0);    }
    float getTemperature()    {  return unpack(temperature, 10, 10);    }

    void setSign() { sign = AQUARIUM_PREAMBLE; }
    bool checkSign() { return sign == AQUARIUM_PREAMBLE; }
    void setCRC() { setSign(); crc = AQUARIUM_checksum((uint8_t *)this, sizeof(*this)-1); }
    bool checkCRC() { return crc == AQUARIUM_checksum((uint8_t *)this, sizeof(*this)-1); }

} DataContainer;

static const uint8_t AQUARIUM_RESPONSE_LENGTH = sizeof(DataContainer);

uint8_t AQUARIUM_checksum(const uint8_t *command, uint8_t count) 
{
    uint8_t sum = 0;
    for (uint8_t i = 1; i < count; i++) 
    {
        sum += command[i];
    }
    return 0xFF - sum + 0x01;
}


}
