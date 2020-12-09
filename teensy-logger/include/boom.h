#include <QuadEncoder.h>

namespace boom
{
	// encode variables
	float current_pos;
	float current_vel;
	float previous_pos = 0;

	int32_t boom_X_CTS;
	const uint32_t pulses_per_revolution = 4096;
	const float XConversion = 0.64; // ???
	const float length_m = 1.75;

	// Enc 1, Phase A (pin0), PhaseB(pin1), Pullups Req(0)
	QuadEncoder encoder(1, 2, 3, 0);

	void init()
	{
		encoder.setInitConfig();
		encoder.EncConfig.revolutionCountCondition = ENABLE;
		encoder.EncConfig.enableModuloCountMode = ENABLE;
		encoder.EncConfig.positionModulusValue = pulses_per_revolution;
		encoder.EncConfig.positionInitialValue = 0;
		encoder.init();

		encoder.write(0);
	}

	float read()
	{
		// get encoder values
		previous_pos = current_pos;

		boom_X_CTS = encoder.read();

		// Check direction
		// if (boom_X_CTS > 2048)
		// {
		// 	boom_X_CTS -= pulses_per_revolution;
		// }
		// else if (boom_X_CTS < -2048)
		// {
		// 	boom_X_CTS += pulses_per_revolution;
		// }

		// Convert to planar
		// TODO: wtf is going on?
		current_pos =
			((length_m) * (boom_X_CTS * M_PI * (44 / 28) / pulses_per_revolution)) / XConversion;

		// // estimate encoder velocity
		// current_vel = (current_pos - previous_pos) / deltaT;
		return current_pos;
	}

} // namespace boom