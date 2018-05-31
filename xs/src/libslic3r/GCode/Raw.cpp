#include "Raw.hpp"

namespace Slic3r {
namespace GCode {
namespace Raw {

std::vector<RawGCodeEvent> discretize_trapezoidal_profiles(const std::vector<TrapezoidalProfile> &moves, MachineLimits &limits)
{
	std::vector<RawGCodeEvent>      events;
	double						    time_prev = 0.;
	for (const TrapezoidalProfile &move : moves) {
		std::vector<RawGCodeEvent> evts = move.events(limits.step_resolution, limits.timer_frequency_Hz);
		for (RawGCodeEvent &e : evts)
			e.time += time_prev;
		events.insert(events.end(), evts.begin(), evts.end());
		time_prev = events.back().time;
	}
	// Reduce to roughly 2x higher accuracy than the maximum chopping frequency of the stepper drivers.
	reduce_events(events, 32. / limits.timer_frequency_Hz);
	return events;
}

void fwrite_gcode_events(FILE *file, std::vector<RawGCodeEvent> &events, const double timer_frequency_Hz)
{
	RawGCodeBlock	block;
	double			time_prev = 0.;
	for (const RawGCodeEvent &evt : events) {
		double time_diff = evt.time - time_prev;
		int    ticks	 = int(floor(timer_frequency_Hz * time_diff + 0.5));
		assert(ticks > 7);
		time_prev += double(ticks) / timer_frequency_Hz;
		// Split the long intervals into filler ticks and the last tick, which will move the axes.
		int    blocks = (ticks + 254) / 255;
		int	   ticks_per_block = (ticks + blocks - 1) / blocks;
		while (ticks) {
			int tick_this = std::min(ticks_per_block, ticks);
			ticks -= tick_this;
			block.add_event(
				(ticks == 0) ? 
					// This is the actual tick.
					evt.bits :
					// This is just a nonzero filler.
					0x10,
				tick_this);
			if (block.full()) {
				block.fwrite(file);
				block.reset();
			}
		}
	}
	if (! block.empty())
		block.fwrite(file);
}

void fwrite_discretize_trapezoidal_profiles(FILE *file, const std::vector<TrapezoidalProfile> &moves, MachineLimits &limits)
{
	if (! moves.empty()) {
		std::vector<RawGCodeEvent> events = discretize_trapezoidal_profiles(moves, limits);
		fwrite_gcode_events(file, events, limits.timer_frequency_Hz);
	}
}

} // namespace Raw
} // namespace GCode
} // namespace Slic3r
