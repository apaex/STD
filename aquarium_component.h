#pragma once

#include "esphome.h"

using namespace aquarium;

namespace esphome
{
	namespace aquarium
	{

		static const char *const TAG = "aquarium";

		class AquariumComponent : public PollingComponent, public UARTDevice
		{
		private:
			int incomingByte = 0;

		public:
			Sensor *pH_sensor = new Sensor();
			Sensor *light_sensor = new Sensor();
			Sensor *level_sensor = new Sensor();
			Sensor *temperature_sensor = new Sensor();

			AquariumComponent(UARTComponent *parent) : PollingComponent(5000), UARTDevice(parent) {}

			float get_setup_priority() const override { return setup_priority::DATA; }

			void setup() override
			{
			}

			void printData(const uint8_t* data, uint8_t size)
			{
				char buff[80];
				for (int i = 0; i < size; ++i) 
				{
					uint8_t d = data[i];
					sprintf(buff + i * 3, "%02X  ", d);
				}
				ESP_LOGD(TAG, "%s", buff);
			}

			void update() override
			{
				CommandContainer command;
				command.command = AQUARIUM_COMMAND_GET_PPM;
				command.setCRC();

				//printData((uint8_t *)&command, AQUARIUM_REQUEST_LENGTH);

				DataContainer response;
				if (!this->AQUARIUM_write_command_((uint8_t *)&command, (uint8_t *)&response))
				{
					ESP_LOGW(TAG, "Reading data from AQUARIUM failed!");
					this->status_set_warning();
					return;
				}

				//printData((uint8_t *)&response, AQUARIUM_RESPONSE_LENGTH);


				if (!response.checkSign())
				{
					ESP_LOGW(TAG, "Invalid preamble from AQUARIUM!");
					this->status_set_warning();
					return;
				}

				if (!response.checkCRC())
				{
					ESP_LOGW(TAG, "AQUARIUM Checksum doesn't match");
					this->status_set_warning();
					return;
				}

				this->status_clear_warning();

				float pH = response.getPH();
				float light = response.getLight();
				float level = response.getLevel();
				float temperature = response.getTemperature();

//				ESP_LOGD(TAG, "AquariumComponent Received pH?=%f light=%flx level=%f%% temp=%f°C", pH, light, level, temperature);

				pH_sensor->publish_state(pH);
				light_sensor->publish_state(light);
				level_sensor->publish_state(level);
				temperature_sensor->publish_state(temperature);
			}

			void dump_config() override
			{
				return PollingComponent::dump_config();
			}

			void loop() override
			{
			}

		protected:
			bool AQUARIUM_write_command_(const uint8_t *command, uint8_t *response)
			{
				// Empty RX Buffer
				while (this->available())
					this->read();
				this->write_array(command, AQUARIUM_REQUEST_LENGTH);
				this->flush();

				if (response == nullptr)
					return true;

				return this->read_array(response, AQUARIUM_RESPONSE_LENGTH);
			}
		};

	} // namespace aquarium
} // namespace esphome