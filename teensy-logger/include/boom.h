#include <QuadEncoder.h>

namespace boom
{
	const uint8_t PHASE_A_PIN = 0;
	const uint8_t PHASE_B_PIN = 1;
	const float length_m = 1.75;

	int32_t boom_encoder_count;

	// QuadEncoder encoder(1, PHASE_A_PIN, PHASE_B_PIN, 0);
	QuadEncoder encoder(1, // channel
						PHASE_A_PIN, PHASE_B_PIN,
						1); // pullups required

	void init()
	{
		encoder.setInitConfig();
		encoder.EncConfig.positionInitialValue = 0;
		encoder.init();

		encoder.write(0);
	}

	int32_t read()
	{
		boom_encoder_count = encoder.read();

		return boom_encoder_count;
	}

} // namespace boom